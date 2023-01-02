#define DEBUG_MOD false //set to true to duplicate the full status in the serial port
#define PRINT_LVL_TEXT false

#define W 15
#define H 10
#define LcdW 16
#define LcdH 2 // if > 2 then refactor LcdDrawMap func
#define LcdRenderW 8 // LcdW / 2
#define ActivateRoomValue 1
#define AxisThreshold 30 // % 
#define MinMaxAxisValues 14000 // max for GY-521 ~ 20000
#define StartMinMaxAxisValues 10000

#define InputDelay 10
#define HeroMoveDelay 300
#define LcdTextPrintDelay 300
#define GameRenderDelay 200 //5 fps
#define GlobalAnimationDelay 500
#define BombExplosionDelay 3000
#define BombExplosionSpreadDelay 500
#define MinimumBombExplosionActionTime 4000

#define LcdFogChar '#' //optionally 255 can be used
#define PortalLeftChars "({"
#define PortalRightChars ")}"
#define BombChars "@ "
#define ExplosionChars "-+*O*O*O*O*O*O"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <LiquidCrystal.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


//-------------------- declare types
enum Room : unsigned char
{
  Closed = 0,
  Right = 1 << 0,
  Down = 1 << 1
};

enum Diraction : unsigned char
{
  Top = 1 << 0,
  RightDiraction = 1 << 1,
  Bottom = 1 << 2,
  Left = 1 << 3,
  ZeroDiraction = 1 << 4,
};

enum ButtonValue : unsigned char
{
  TopButton = 1 << 0,
  RightButton = 1 << 1,
  BottomButton = 1 << 2,
  LeftButton = 1 << 3,
  SelectButton = 1 << 4,
  NoneButton = 1 << 5,
};

enum AppState : unsigned char
{
  PrintInfo = 0,
  GameLoop = 1
};

struct Point
{
  unsigned char X;
  unsigned char Y;
};

struct PointDiraction
{
  Point P;
  Diraction Diraction;
};
//-------------------- end declare types


//-------------------- common
template<typename T>
void ClearArr(T arr[], int count, T defaultVal)
{
  for (int i = 0; i < count; ++i)
    arr[i] = defaultVal;
}

char* StringConcatenate(const char *first, const char *second) 
{
    int l1 = 0, l2 = 0;
    const char * f = first, * l = second;

    while (*f++) ++l1;
    while (*l++) ++l2;

    char *result = new char[l1 + l2 + 1];

    for (int i = 0; i < l1; i++) result[i] = first[i];
    for (int i = l1; i < l1 + l2; i++) result[i] = second[i - l1];

    result[l1 + l2] = '\0';
    return result;
}
//-------------------- end common


//-------------------- animation container
template<typename T> class AnimationContainer
{
  private:
    T *_animList;
    byte _count;
    byte _currentIndex;
    byte _size;
    unsigned short _speedSwichFrame;
    unsigned long _lastExecTime;

  public:
    AnimationContainer(T *animList, byte count, byte size, unsigned short speedSwichFrame)
    {
      _animList = animList;
      _count = count;
      _size = size;      
      _speedSwichFrame = speedSwichFrame;

      Reset();
    }

    void Reset()
    {
      _currentIndex = 0;
      _lastExecTime = millis();
    }

    void IncrementIndex()
    {
      unsigned long currentTime = millis();

      if ((currentTime - _lastExecTime) > _speedSwichFrame)
      {
        _currentIndex = ++_currentIndex % _count;
        _lastExecTime = currentTime;
      }      
    }

    T* Current()
    {
      return &_animList[_currentIndex * _size];
    }
};
//-------------------- end animation container


//-------------------- time worker
class TimeWorker
{
  private:
    unsigned short _delay;
    void (*_clbk)(bool);
    unsigned long _lastExecTime;
    bool *_eventInvokeFlag;
    bool _onlyEventInvoked;

  public:
    //eventInvokeFlag - clbk вызывается сразу же, когда eventInvokeFlag = true.
    //onlyEventInvoked - Если true, делает вызов clbk только по eventInvokeFlag, при этом между вызовами должно пройти delay или больше времени.
    //  Если false, вызывает постоянно дополнительно по таймеру.
    TimeWorker(unsigned short delay, void (*clbk)(bool), bool *eventInvokeFlag = NULL, bool onlyEventInvoked = true)
    {
      _delay = delay;
      _clbk = clbk;
      _lastExecTime = millis();
      _eventInvokeFlag = eventInvokeFlag;
      _onlyEventInvoked = onlyEventInvoked;
    }

    void Update()
    {
      unsigned long currentTime = millis();
      bool invoke = (currentTime - _lastExecTime) > _delay;
      bool clbkArg = false;

      if (_eventInvokeFlag != NULL)
      {
        invoke = (invoke && _onlyEventInvoked && (*_eventInvokeFlag)) || (!_onlyEventInvoked && (invoke || (*_eventInvokeFlag)));
        clbkArg = (*_eventInvokeFlag);
        *_eventInvokeFlag = false;
      }

      if (invoke)
      {
        (*_clbk)(clbkArg);
        _lastExecTime = currentTime;
      }
    }

    void SetOnlyEventInvoked(bool value)
    {
      _onlyEventInvoked = value;
    }
};
//-------------------- end time worker


//-------------------- stack
template<typename T> class Node
{
  public:
    T Value;
    Node* NextNode;
    Node(const T value, Node* nextNode)
    {
      Value = value;
      NextNode = nextNode;
    }
};

template<typename T> class NodeIterator
{
  private:
    Node<T>* _currentNode;

  public:
    NodeIterator(Node<T> *startNode)
    {
      _currentNode = startNode;
    }

    bool MoveNext()
    {
      if (_currentNode->NextNode == NULL)
        return false;

      _currentNode = _currentNode->NextNode;
      return true;
    }

    T GetValue()
    {
      return _currentNode->Value;
    }

    void SetValue(T value)
    {
      _currentNode->Value = value;
    }
};

template<typename T> class Stack
{
  private:
    Node<T>* _currentNode;
    unsigned int _count;
    T _default;

  public:
    Stack()
    {
      _currentNode = NULL;
      _count = 0;
    }

    void Clear()
    {
      while (_count != 0)
      {
        Pop();
      }
    }

    ~Stack()
    {
      Clear();
    }

    void Push (const T item)
    {
      Node<T>* newNode = new Node<T>(item, NULL);
      
      if (_count == 0)
      {
        _currentNode = newNode;
      }
      else
      {
        newNode->NextNode = _currentNode;
        _currentNode = newNode;
      }

      _count++;
    }
    
    T Pop()
    {
      if (_count == 0)
        return _default;

      Node<T>* current = _currentNode;
      T currentVal = current->Value;
      _currentNode = current->NextNode;
      _count--;
      delete current;

      return currentVal;
    }

    bool Contains(T val)
    {
      if (_count == 0)
        return false;

      Node<T>* node = _currentNode;
      while (node != NULL)
      {
        if (memcmp(&node->Value, &val, sizeof(T)) == 0)
          return true;

        node = node->NextNode;
      }

      return false;
    }

    unsigned int Count()
    {
      return _count;
    }

    NodeIterator<T>* CreateIterator()
    {
      return new NodeIterator<T>(_currentNode);
    }
};
//-------------------- end stack


//-------------------- lib vars
MPU6050 CY531;
LiquidCrystal Lcd(8, 9, 4, 5, 6, 7);
//-------------------- end lib vars


//-------------------- axis
int16_t MaxAxisX, MinAxisX, MaxAxisY, MinAxisY, LastFilterX, LastFilterY;

bool InitAxis()
{
  MaxAxisY = StartMinMaxAxisValues;
  MinAxisY = -StartMinMaxAxisValues;
  MaxAxisX = StartMinMaxAxisValues;
  MinAxisX = -StartMinMaxAxisValues;
  Wire.begin();
  CY531.initialize();
  delay(100);

  if (!CY531.testConnection())
  {
    Serial.println("Axis not connected");
    Lcd.print("Axis not connected");
    delay(500);
    return false;
  }

  return true;
}

int16_t GetAxisDiff(int16_t max, int16_t min, int16_t current)
{
  int16_t calcThreshold = ((long)max - min) / 100. * AxisThreshold;

  if (abs(current) >= calcThreshold)
    return current;

  return 0;
}

int16_t FilterAxisVal(int16_t *lastFilterValue, int16_t *currentValue)
{
  *currentValue = (*currentValue) * 0.3 + (*lastFilterValue) * 0.7;
  *lastFilterValue = *currentValue;
}

Diraction GetAxisDiraction()
{
  int16_t ax, ay, az;
  CY531.getAcceleration(&ax, &ay, &az);
  FilterAxisVal(&LastFilterX, &ax);
  FilterAxisVal(&LastFilterY, &ay);

  //Serial.print("max:"); Serial.print(MinMaxAxisValues);
  //Serial.print(",min:"); Serial.print(-MinMaxAxisValues);
  //Serial.print(",x:"); Serial.print(ax);
  //Serial.print(",y:"); Serial.println(-ay);

  if (ax > MaxAxisX) MaxAxisX = ax;
  if (abs(MaxAxisX) > MinMaxAxisValues) MaxAxisX = MinMaxAxisValues;
  if (ax < MinAxisX) MinAxisX = ax;
  if (abs(MinAxisX) > MinMaxAxisValues) MinAxisX = MinMaxAxisValues;
  if (ay > MaxAxisY) MaxAxisY = ay;
  if (abs(MaxAxisY) > MinMaxAxisValues) MaxAxisY = MinMaxAxisValues;
  if (ay < MinAxisY) MinAxisY = ay;
  if (abs(MinAxisY) > MinMaxAxisValues) MinAxisY = MinMaxAxisValues;
  
  int16_t diffX = GetAxisDiff(MaxAxisX, MinAxisX, ax);
  int16_t diffY = GetAxisDiff(MaxAxisY, MinAxisY, ay);

  if (diffX > 0)
    return Bottom;
  else if (diffX < 0)
    return Top;

  if (diffY > 0)
    return RightDiraction;
  else if (diffY < 0)
    return Left;

  return ZeroDiraction;
}
//-------------------- end axis


//-------------------- global game vars
Room Map[H][W];
Point Hero = { .X = 0, .Y = 0 };
bool HeroPosIsRightChar = false;
Point Portal = { .X = 0, .Y = 0 };
Diraction LastAxisDiraction = ZeroDiraction;
AppState CurrentAppState = PrintInfo;
unsigned int LvlCounter = 0;
ButtonValue Button = NoneButton;

//--BOMB--
Point *Bomb = NULL;
bool BombIsRight = false;
bool BombIsExplosion = false;
Stack<Point> *ExplosionPoints = NULL;
Point ExplosionTop = { .X = 0, .Y = 0 };
Point ExplosionRight = { .X = 0, .Y = 0 };
Point ExplosionLeft = { .X = 0, .Y = 0 };
Point ExplosionBottom = { .X = 0, .Y = 0 };
//-------------------- end global game vars


//-------------------- map
void GenerateMap()
{
  Stack<PointDiraction>* stack = new Stack<PointDiraction>();
  PointDiraction empty { .P = { .X = 0, .Y = 0 }, .Diraction = ZeroDiraction };

  stack->Push({ .P = { .X = 0, .Y = 0 }, .Diraction = Left});
  while (stack->Count() > 0)
  {    
    PointDiraction pd = stack->Pop();

    if (RoomIsAvailable(pd.P))
    {
      switch (pd.Diraction)
      {
        case Top:
          RoomAddValue(pd.P, Down);
          break;
        case RightDiraction:
          RoomAddValue({ .X = pd.P.X - 1, .Y = pd.P.Y }, Right);
          break;
        case Bottom:
          RoomAddValue({ .X = pd.P.X, .Y = pd.P.Y - 1 }, Down);
          break;
        case Left:
          RoomAddValue(pd.P, Right);
          break;
      }

      int pointsIndex = 0;
      PointDiraction points[4] { empty, empty, empty, empty };
      if (pd.P.X - 1 > -1)
        points[pointsIndex++] = { .P = { .X = pd.P.X - 1, pd.P.Y }, .Diraction = Left };
      if (pd.P.X + 1 < W)
        points[pointsIndex++] = { .P = { .X = pd.P.X + 1, pd.P.Y }, .Diraction = RightDiraction };
      if (pd.P.Y - 1 > -1)
        points[pointsIndex++] = { .P = { .X = pd.P.X, pd.P.Y - 1 }, .Diraction = Top };
      if (pd.P.Y + 1 < H)
        points[pointsIndex++] = { .P = { .X = pd.P.X, pd.P.Y + 1 }, .Diraction = Bottom };

      int n = pointsIndex;
      while (n > 1)
      {
        int k = random(n--);
        PointDiraction temp = points[n];
        points[n] = points[k];
        points[k] = temp;
      }

      for (int i = 0; i < pointsIndex; ++i)
      {
        if (!stack->Contains(points[i]) && RoomIsAvailable(points[i].P))
          stack->Push(points[i]); 
      }
    }
  }

  delete stack;
}

bool RoomIsRight(Room room)
{
  return (room & Right) == Right;
}

bool RoomIsDown(Room room)
{
  return (room & Down) == Down;
}

void RoomAddValue(Point p, Room val)
{
  Map[p.Y][p.X] |= val;
}

bool RoomIsAvailable(Point p)
{
  int sides = 0;

  if (p.X - 1 >= 0 && RoomIsRight(Map[p.Y][p.X - 1]))
    sides++;
  if (RoomIsRight(Map[p.Y][p.X]))
    sides++;
  if (p.Y - 1 >= 0 && RoomIsDown(Map[p.Y - 1][p.X]))
    sides++;
  if (RoomIsDown(Map[p.Y][p.X]))
    sides++;  

  return sides < ActivateRoomValue;
}

void ClearMap() 
{
  for (int y = 0; y < H; ++y)
  {
    for (int x = 0; x < W; ++x)
    {
      Map[y][x] = Closed;
    }  
  }
}

void DrawMap()
{
  for (int i = 0; i < W; ++i)
  {
    Serial.print(" __");
  }  

  Serial.print("\n");

  for (int y = 0; y < H; ++y)
  {
    Serial.print("|");
    for (int x = 0; x < W; ++x)
    {
      Room val = Map[y][x];
      char* offset = "  ";

      if (Hero.Y == y && Hero.X == x)
      {
        if (HeroPosIsRightChar)
          Serial.print(" #");
        else
          Serial.print("# ");
        offset = "";
      }
      else if (Portal.Y == y && Portal.X == x)
      {
        Serial.print("()");
        offset = "";
      }

      Serial.print(offset);

      if (val & Right)
        Serial.print(" ");
      else
        Serial.print("|");
    }
    Serial.print("\n");

    Serial.print("|");
    for (int x = 0; x < W; ++x)
    {
      Room val = Map[y][x];

      if (val & Down)
        Serial.print("  ");
      else
        Serial.print("__");

      if (val & Right)
        Serial.print(" ");
      else
        Serial.print("|");
    }
    Serial.print("\n");
  }
}

void PrintHeroAndPortalCoord()
{
  Serial.print("Hero "); Serial.print(Hero.X); Serial.print(":"); Serial.print(Hero.Y);
  Serial.print("  Portal "); Serial.print(Portal.X); Serial.print(":"); Serial.println(Portal.Y);
}

bool CheckTopRoomAvailable(unsigned char x, unsigned char y)
{
  if ((y - 1) > -1 && RoomIsDown(Map[y - 1][x]))
  {
    return true;
  }

  return false;
}

bool CheckRightRoomAvailable(unsigned char x, unsigned char y)
{
  if ((x + 1) < W && RoomIsRight(Map[y][x]))
  {
    return true;
  }

  return false;
}

bool CheckBottomRoomAvailable(unsigned char x, unsigned char y)
{
  if ((y + 1) < H && RoomIsDown(Map[y][x]))
  {
    return true;
  }

  return false;
}

bool CheckLeftRoomAvailable(unsigned char x, unsigned char y)
{
  if ((x - 1) > -1 && RoomIsRight(Map[y][x - 1]))
  {
    return true;
  }

  return false;
}

Diraction GetRoomDiractions(unsigned char x, unsigned char y)
{
  Diraction res = ZeroDiraction;

  if (!CheckTopRoomAvailable(x, y))
    res |= Top;

  if (!CheckBottomRoomAvailable(x, y))
    res |= Bottom;

  if (!CheckRightRoomAvailable(x, y))
    res |= RightDiraction;

  if (!CheckLeftRoomAvailable(x, y))
    res |= Left;

  return res;
}

void InitPortalPoint()
{ 
  Portal.X = random(W);
  Portal.Y = random((Portal.X >= (W / 2) ? 0 : H / 2), H);
}
//-------------------- end map


//-------------------- hero
bool HeroMoveTo(Diraction diraction)
{
  if (diraction == Top)
  {
    if ((Hero.Y - 1) > -1 && RoomIsDown(Map[Hero.Y - 1][Hero.X]))
    {
      Hero.Y -= 1;
      return true;
    }  
  }
  else if (diraction == RightDiraction)
  {
    if (!HeroPosIsRightChar)
    {
      HeroPosIsRightChar = true;
      return true;
    }
    else if ((Hero.X + 1) < W && RoomIsRight(Map[Hero.Y][Hero.X]))
    {
      Hero.X += 1;
      HeroPosIsRightChar = false;
      return true;
    }
  }
  else if (diraction == Bottom)
  {
    if ((Hero.Y + 1) < H && RoomIsDown(Map[Hero.Y][Hero.X]))
    {
      Hero.Y += 1;
      return true;
    }
  }
  else if (diraction == Left)
  {
    if (HeroPosIsRightChar)
    {
      HeroPosIsRightChar = false;
      return true;
    }
    else if ((Hero.X - 1) > -1 && RoomIsRight(Map[Hero.Y][Hero.X - 1]))
    {
      Hero.X -= 1;
      HeroPosIsRightChar = true;
      return true;
    }
  }

  return false;
}

bool ExplosionPropagation()
{
  bool propagate = false;

  if (CheckTopRoomAvailable(ExplosionTop.X, ExplosionTop.Y))
  {
    ExplosionTop.Y--;
    ExplosionPoints->Push(ExplosionTop);
    propagate = true;
  }

  if (CheckRightRoomAvailable(ExplosionRight.X, ExplosionRight.Y))
  {
    ExplosionRight.X++;
    ExplosionPoints->Push(ExplosionRight);
    propagate = true;
  }

  if (CheckBottomRoomAvailable(ExplosionBottom.X, ExplosionBottom.Y))
  {
    ExplosionBottom.Y++;
    ExplosionPoints->Push(ExplosionBottom);
    propagate = true;
  }

  if (CheckLeftRoomAvailable(ExplosionLeft.X, ExplosionLeft.Y))
  {
    ExplosionLeft.X--;
    ExplosionPoints->Push(ExplosionLeft);
    propagate = true;
  }

  return propagate;
}
//-------------------- end hero


//-------------------- lcd drawer
const byte LeftWall[8] = 
{
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
  B10000,
};

const byte RightWall[8] = 
{
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
  B00001,
};

const byte TopWall[8] = 
{
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
};

const byte BottomWall[8] = 
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
};

const byte HeroChars[2][8] = 
{
  {
    B00000,
    B00000,
    B01110,
    B01010,
    B00100,
    B01010,
    B00000,
    B00000, 
  },
  {
    B00000,
    B00000,
    B01110,
    B01010,
    B00100,
    B00100,
    B00000,
    B00000, 
  }
};

AnimationContainer<byte> HeroAnimation = AnimationContainer<byte>(HeroChars[0], 2, sizeof(byte) * 8, GlobalAnimationDelay);
AnimationContainer<char> PortalLeftAnimation = AnimationContainer<char>(PortalLeftChars, 2, sizeof(char), GlobalAnimationDelay);
AnimationContainer<char> PortalRightAnimation = AnimationContainer<char>(PortalRightChars, 2, sizeof(char), GlobalAnimationDelay);
AnimationContainer<char> BombAnimation = AnimationContainer<char>(BombChars, 2, sizeof(char), GlobalAnimationDelay / 2);
AnimationContainer<char> ExplosionAnimation = AnimationContainer<char>(ExplosionChars, 14, sizeof(char), BombExplosionSpreadDelay);

void InitLcd()
{
  Lcd.begin(LcdH, LcdH);
  Lcd.noCursor();
  Lcd.noBlink();
  Lcd.noAutoscroll();
  Lcd.clear();
}

void MergeChars(byte target[], byte source[])
{
  for (int i = 0; i < 8; ++i)
    target[i] |= source[i];
}

void CreateRoomLeftChar(byte target[], Diraction diraction)
{
  if ((diraction & Top) == Top)
    MergeChars(target, TopWall);

  if ((diraction & Bottom) == Bottom)
    MergeChars(target, BottomWall);

  if ((diraction & Left) == Left)
    MergeChars(target, LeftWall);
}

void CreateRoomRightChar(byte target[], Diraction diraction)
{
  if ((diraction & Top) == Top)
    MergeChars(target, TopWall);

  if ((diraction & Bottom) == Bottom)
    MergeChars(target, BottomWall);

  if ((diraction & RightDiraction) == RightDiraction)
    MergeChars(target, RightWall);
}

bool EqualChars(byte char1[], byte char2[])
{
  return memcmp(char1, char2, 8) == 0;
}

byte FindIndexChar(byte chars[][8], byte char1[], byte maxChars)
{
  for (int i = 0; i < maxChars; ++i)
  {    
    if (EqualChars(chars[i], char1))
      return i;
  }

  return 255;
}

void LcdCacheCreateChar(unsigned char x, unsigned char y, byte lcdIndexes[], byte chars[][8], uint8_t lcdX, byte *maxCharIndex, Diraction charExist)
{
  Diraction currentPos = GetRoomDiractions(x, y);
  
  CreateRoomLeftChar(chars[*maxCharIndex], currentPos);
  byte findIndex = FindIndexChar(chars, chars[*maxCharIndex], *maxCharIndex);
  if (findIndex == 255)
  {
    findIndex = *maxCharIndex;
    (*maxCharIndex)++;
  }
  else
  {
    ClearArr<byte>(chars[*maxCharIndex], 8, 0);
  }

  if ((charExist & Left) != Left)
  {
    lcdIndexes[lcdX * 2] = findIndex;
  }
  
  CreateRoomRightChar(chars[*maxCharIndex], currentPos);
  if (EqualChars(chars[*maxCharIndex], chars[findIndex]))
  {
    ClearArr<byte>(chars[*maxCharIndex], 8, 0);
  }
  else
  {
    findIndex = FindIndexChar(chars, chars[*maxCharIndex], *maxCharIndex);

    if (findIndex == 255)
    {
      findIndex = *maxCharIndex;
      (*maxCharIndex)++;
    }
    else
    {
      ClearArr<byte>(chars[*maxCharIndex], 8, 0);
    }    
  } 

  if ((charExist & RightDiraction) != RightDiraction)
  {
    lcdIndexes[lcdX * 2 + 1] = findIndex;
  }
}

void ReconfigureFirstHeroChar(byte lcdIndexes[], byte chars[][8], uint8_t lcdX, byte *maxCharIndex)
{
  int lcdIndex = lcdX * 2;
  bool rightFree = lcdIndexes[lcdIndex + 1] < 8;
  bool allFree = lcdIndexes[lcdIndex] < 8 && rightFree;

  if ((*maxCharIndex) < 2)
  {
    (*maxCharIndex)++;
    MergeChars(chars[1], chars[0]);

    if (rightFree)
      lcdIndexes[lcdIndex + 1] = 1;
  }

  MergeChars(chars[(allFree || (*maxCharIndex) == 2 ? (int)HeroPosIsRightChar : 0)], HeroAnimation.Current());
}

bool LcdCheckExplosionPoint(unsigned char x, unsigned char y)
{
  NodeIterator<Point> *iterator = ExplosionPoints->CreateIterator();

  do
  {
    Point p = iterator->GetValue();
    if (p.X == x && p.Y == y)
    {
      delete iterator;
      return true;
    }
  } 
  while (iterator->MoveNext());       

  delete iterator;
  return false;
}

Diraction LcdCacheCreateCharCheckExistingGameObjs(unsigned char x, unsigned char y, byte lcdIndexes[], int leftIndex, int rightIndex)
{
  byte bombChar = (byte)*BombAnimation.Current();

  if (x == Portal.X && y == Portal.Y)
  {
    lcdIndexes[leftIndex] = (byte)*PortalLeftAnimation.Current();
    lcdIndexes[rightIndex] = (byte)*PortalRightAnimation.Current();
  }
  else if (Bomb != NULL && (bombChar != (byte)' ' || BombIsExplosion) && x == Bomb->X && y == Bomb->Y)
  {
    lcdIndexes[BombIsRight ? rightIndex : leftIndex] = bombChar;

    if (BombIsExplosion)
    {
      lcdIndexes[BombIsRight ? leftIndex : rightIndex] = (byte)*ExplosionAnimation.Current();

      if (bombChar == (byte)' ')
        return BombIsRight ? Left : RightDiraction;
    }
    else
    {
      return BombIsRight ? RightDiraction : Left;
    }
  }
  else if (ExplosionPoints != NULL && ExplosionPoints->Count() > 0 && LcdCheckExplosionPoint(x, y))
  {
    byte explosionChar = (byte)*ExplosionAnimation.Current();

    lcdIndexes[leftIndex] = explosionChar;
    lcdIndexes[rightIndex] = explosionChar;
  }
  else
  {
    return ZeroDiraction;
  }

  return RightDiraction | Left;
}

bool LcdCacheCreateCharCheckExistingGameObjsOrDefault(unsigned char x, unsigned char y, byte lcdIndexes[], byte chars[][8], byte *maxCharIndex)
{
  uint8_t nLcdX = x % LcdRenderW;
  Diraction existingSet = LcdCacheCreateCharCheckExistingGameObjs(x, y, lcdIndexes, nLcdX * 2, nLcdX * 2 + 1);
  LcdCacheCreateChar(x, y, lcdIndexes, chars, nLcdX, maxCharIndex, existingSet);

  return existingSet == (RightDiraction | Left);
}

void LcdDrawMap(unsigned char x, unsigned char y)
{
  uint8_t lcdY = y % LcdH;
  uint8_t lcdX = x % LcdRenderW;

  byte lcdIndexes[LcdW];
  ClearArr<byte>(lcdIndexes, LcdW, 255);

  byte maxCharIndex = 0;
  byte chars[8][8];
  for (int i = 0; i < 8; ++i)
    ClearArr<byte>(chars[i], 8, 0);

  if (!LcdCacheCreateCharCheckExistingGameObjsOrDefault(x, y, lcdIndexes, chars, &maxCharIndex))
    ReconfigureFirstHeroChar(lcdIndexes, chars, lcdX, &maxCharIndex);

  unsigned char newX = x;
  while (CheckLeftRoomAvailable(newX--, y) && (newX % LcdRenderW) < lcdX)
  {
    LcdCacheCreateCharCheckExistingGameObjsOrDefault(newX, y, lcdIndexes, chars, &maxCharIndex);
  }

  newX = x;
  while (CheckRightRoomAvailable(newX++, y) && (newX % LcdRenderW) > lcdX)
  {
    LcdCacheCreateCharCheckExistingGameObjsOrDefault(newX, y, lcdIndexes, chars, &maxCharIndex);
  }

  byte lcd2RowIndexes[2] = { (byte)LcdFogChar, (byte)LcdFogChar };
  if (lcdY == 0 && CheckBottomRoomAvailable(x, y) || lcdY == 1 && CheckTopRoomAvailable(x, y))
  {
    unsigned char newY = y + (lcdY == 0 ? 1 : -1);
    Diraction existingSet = LcdCacheCreateCharCheckExistingGameObjs(x, newY, lcd2RowIndexes, 0, 1);
    LcdCacheCreateChar(x, newY, lcd2RowIndexes, chars, 0, &maxCharIndex, existingSet);
  }
  
  for (int i = 0; i < maxCharIndex; ++i)
  {
    Lcd.createChar(i, chars[i]);
  }

  for (int i = 0; i < LcdW; ++i)
  {
    Lcd.setCursor((uint8_t)i, lcdY);
    if (lcdIndexes[i] < 255)
      Lcd.write(lcdIndexes[i]);
    else
      Lcd.write((byte)LcdFogChar);
  }

  uint8_t otherLcdY = lcdY == 0 ? 1 : 0;
  byte lcdStartX = lcdX * 2;

  Lcd.setCursor(0, otherLcdY);
  for (byte i = 0; i < lcdStartX; ++i)
  {
    Lcd.write((byte)LcdFogChar);
  }

  Lcd.setCursor(lcdStartX + 2, otherLcdY);
  for (byte i = lcdStartX + 2; i < LcdW; ++i)
  {
    Lcd.write((byte)LcdFogChar);
  }

  Lcd.setCursor(lcdX * 2, otherLcdY);
  Lcd.write(lcd2RowIndexes[0]);
  Lcd.write(lcd2RowIndexes[1]);
}

char *LcdText = NULL;
int TextIndex = 0;
int TextOffSet = 0;

void InitLcdText()
{
  Lcd.clear();

  char *depth = "DEPTH ";
  char *goodluck = "\ngood luck...  ";
  String *lvlCounterStr = new String(LvlCounter);
  char *temp = StringConcatenate(depth, lvlCounterStr->c_str());
  
  LcdText = StringConcatenate(temp, goodluck);

  delete lvlCounterStr;
  delete temp;
}

bool DrawChar()
{
  if (LcdText != NULL && LcdText[TextIndex] != '\0')
  {
    if (LcdText[TextIndex] == '\n')
    {
#if DEBUG_MOD
      Serial.println();
#endif

      TextOffSet = ++TextIndex;
    }
    else
    {
#if DEBUG_MOD
      Serial.print(LcdText[TextIndex]);
#endif

      Lcd.setCursor(TextIndex - TextOffSet, TextOffSet == 0 ? 0 : 1);
      Lcd.print(LcdText[TextIndex++]);
    }    

    return true;
  }  
#if DEBUG_MOD
  else if (LcdText[TextIndex] == '\0')
  {
    Serial.println();
  }
#endif

  char *ptr = LcdText;
  LcdText = NULL;
  delete ptr;
  TextIndex = 0;
  TextOffSet = 0;
  return false;  
}

#if DEBUG_MOD
void DrawGame(bool eventExec)
{
  if (eventExec)
  {
    DrawMap();
    PrintHeroAndPortalCoord();
  }    
#else
void DrawGame()
{
#endif    
    
  LcdDrawMap(Hero.X, Hero.Y);
}
//-------------------- end lcd drawer


//-------------------- workers
bool InvokeHeroMoveWorkerFlag = false;
bool InvokeGameRenderWorkerFlag = false;
bool InvokeHeroBombWorkerFlag = false;
bool BombExplosionWorkerFlag = false;
byte BombExplosionWorkerCounter = 0;


void InputWorkerClbk(bool eventExec)
{
  LastAxisDiraction = GetAxisDiraction();

  if (LastAxisDiraction != ZeroDiraction)
    InvokeHeroMoveWorkerFlag = true;

  int buttonValue = analogRead(0);
  if (buttonValue < 100) 
    Button = TopButton;
  else if (buttonValue < 200) 
    Button = TopButton;
  else if (buttonValue < 400)
    Button = BottomButton;
  else if (buttonValue < 600)
    Button = LeftButton;
  else if (buttonValue < 800)
    Button = SelectButton;
  else
    Button = NoneButton;
}
TimeWorker InputWorker = TimeWorker(InputDelay, InputWorkerClbk);


void GameRenderWorkerClbk(bool eventExec)
{
#if DEBUG_MOD
  DrawGame(eventExec);   
#else
  DrawGame();
#endif
}
TimeWorker GameRenderWorker = TimeWorker(GameRenderDelay, GameRenderWorkerClbk, &InvokeGameRenderWorkerFlag, false);


void HeroMoveWorkerClbk(bool eventExec)
{
  Point pastHero = Hero;
  bool pastHeroPosIsRightChar = HeroPosIsRightChar;

  if (Hero.X == Portal.X && Hero.Y == Portal.Y)
  {    
    LvlCounter++;
    Hero.X = 0;
    Hero.Y = 0;

    InitLcdText();
    GameRenderWorker.SetOnlyEventInvoked(true);

    CurrentAppState = PrintInfo;
  }
  else if (HeroMoveTo(LastAxisDiraction))
  {
    if (Bomb != NULL && Hero.X == Bomb->X && Hero.Y == Bomb->Y && HeroPosIsRightChar == BombIsRight)
    {
      Hero = pastHero;
      HeroPosIsRightChar = pastHeroPosIsRightChar;
    }
    else
    {
      InvokeGameRenderWorkerFlag = true;
    }
  }
}
TimeWorker HeroMoveWorker = TimeWorker(HeroMoveDelay, HeroMoveWorkerClbk, &InvokeHeroMoveWorkerFlag);


void BombExplosionWorkerClbk(bool eventExec)
{
  if (!BombIsExplosion)
    return;

  if (!ExplosionPropagation() && (BombExplosionSpreadDelay * BombExplosionWorkerCounter) >= MinimumBombExplosionActionTime)
  {
    BombIsExplosion = false;

    Point *currentBomb = Bomb;
    Bomb = NULL;
    delete currentBomb;

    Stack<Point> *currentExplosionPoints = ExplosionPoints;
    ExplosionPoints->Clear();
    ExplosionPoints = NULL;
    delete currentExplosionPoints;
  }
  else
  {
    BombExplosionWorkerCounter++;
  }
}
TimeWorker BombExplosionWorker = TimeWorker(BombExplosionSpreadDelay, BombExplosionWorkerClbk, &BombExplosionWorkerFlag, false);


void HeroBombWorkerClbk(bool eventExec)
{
  if (eventExec)
    return;

  if (!BombIsExplosion)
  {
    BombIsExplosion = true;
    BombExplosionWorkerFlag = true;
    ExplosionPoints = new Stack<Point>();
    ExplosionTop = *Bomb;
    ExplosionRight = ExplosionTop;
    ExplosionLeft = ExplosionTop;
    ExplosionBottom = ExplosionTop;
    ExplosionAnimation.Reset();
    BombExplosionWorkerCounter = 0;
  }  
}
TimeWorker HeroBombWorker = TimeWorker(BombExplosionDelay, HeroBombWorkerClbk, &InvokeHeroBombWorkerFlag, false);


void GameLogicWorkerClbk(bool eventExec)
{ 
  HeroMoveWorker.Update();

  if (Button == SelectButton && Bomb == NULL)
  {
    BombIsRight = !HeroPosIsRightChar;
    Bomb = new Point();
    Bomb->X = Hero.X;
    Bomb->Y = Hero.Y;

    InvokeHeroBombWorkerFlag = true;
    InvokeGameRenderWorkerFlag = true;
  }
  else if (Bomb != NULL)
  {
    HeroBombWorker.Update();
    BombExplosionWorker.Update();
  }
}
TimeWorker GameLogicWorker = TimeWorker(InputDelay, GameLogicWorkerClbk);


void LcdTextPrintWorkerClbk(bool eventExec)
{
#if PRINT_LVL_TEXT
  if (!DrawChar())
#else
  if (true)
#endif
  {
    ClearMap();
    GenerateMap();
    InitPortalPoint();
    GameRenderWorker.SetOnlyEventInvoked(false);

    CurrentAppState = GameLoop;
  }
}
TimeWorker LcdTextPrintWorker = TimeWorker(LcdTextPrintDelay, LcdTextPrintWorkerClbk);


void GlobalAnimationWorkerClbk(bool eventExec)
{
  PortalLeftAnimation.IncrementIndex();
  PortalRightAnimation.IncrementIndex();
  HeroAnimation.IncrementIndex();
  BombAnimation.IncrementIndex();
  ExplosionAnimation.IncrementIndex();
}
TimeWorker GlobalAnimationWorker = TimeWorker(GameRenderDelay, GlobalAnimationWorkerClbk);
//-------------------- end workers


//-------------------- main
void ArduinoOff()
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_mode();
}

void setup() 
{
  Serial.begin(9600);
  randomSeed(analogRead(1));
  InitLcd();
  if (!InitAxis())
  {
    ArduinoOff(); 
    return;
  }

  CurrentAppState = PrintInfo;
  InitLcdText();
}

void loop() 
{
  InputWorker.Update();

  switch (CurrentAppState)
  {
    case GameLoop:
      GameLogicWorker.Update();
      GlobalAnimationWorker.Update();
      GameRenderWorker.Update();
    break;
    case PrintInfo:
      LcdTextPrintWorker.Update();
    break;
  }
}
//-------------------- end main
