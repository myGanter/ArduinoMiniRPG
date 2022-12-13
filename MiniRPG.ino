#define DEBUG_MOD true //set to true to duplicate the full status in the serial port
#define PRINT_LVL_TEXT true
#define W 10
#define H 10
#define InputDelay 10
#define GameLoopDelay 300
#define LcdTextPrintDelay 300
#define GameRenderDelay 125 //8 fps
#define GlobalAnimationDelay 500
#define ActivateRoomValue 1
#define AxisThreshold 30 // % 
#define MinMaxAxisValues 14000 // max for GY-521 ~ 20000
#define StartMinMaxAxisValues 10000
#define LcdW 16
#define LcdH 2 // if > 2 then refactor LcdDrawMap func
#define LcdRenderW 8 // LcdW / 2
#define LcdFogChar '#' //optionally 255 can be used
#define PortalLeftChars "({"
#define PortalRightChars ")}"

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
  
  public:
    AnimationContainer(T *animList, byte count)
    {
      _animList = animList;
      _count = count;
      _currentIndex = 0;
    }

    void IncrementIndex()
    {
      _currentIndex = ++_currentIndex % _count;
    }

    T Current()
    {
      return _animList[_currentIndex];
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
Point Portal = { .X = 0, .Y = 0 };
bool HeroPosIsRightChar = false;
Diraction LastAxisDiraction = ZeroDiraction;
AppState CurrentAppState = PrintInfo;
unsigned int LvlCounter = 0;
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

const byte HeroChar[8] = 
{
  B00000,
  B00000,
  B00010,
  B00100,
  B00100,
  B01010,
  B00000,
  B00000,
};

AnimationContainer<char> PortalLeftAnimation = AnimationContainer<char>(PortalLeftChars, 2);
AnimationContainer<char> PortalRightAnimation = AnimationContainer<char>(PortalRightChars, 2);

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

void LcdCacheCreateChar(unsigned char x, unsigned char y, byte lcdIndexes[], byte chars[][8], uint8_t lcdX, byte *maxCharIndex)
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

  lcdIndexes[lcdX * 2] = findIndex;

  CreateRoomRightChar(chars[*maxCharIndex], currentPos);
  if (EqualChars(chars[*maxCharIndex], chars[findIndex]))
  {
    ClearArr<byte>(chars[*maxCharIndex], 8, 0);

    lcdIndexes[lcdX * 2 + 1] = findIndex;
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

    lcdIndexes[lcdX * 2 + 1] = findIndex;
  } 
}

void ReconfigureFirstHeroChar(byte lcdIndexes[], byte chars[][8], uint8_t lcdX, byte *maxCharIndex)
{
  if ((*maxCharIndex) < 2)
  {
    (*maxCharIndex)++;
    MergeChars(chars[1], chars[0]);
    lcdIndexes[lcdX * 2 + 1] = 1;
  }

  MergeChars(chars[(int)HeroPosIsRightChar], HeroChar);
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

  if (x == Portal.X && y == Portal.Y)
  {
    lcdIndexes[lcdX * 2] = (byte)PortalLeftAnimation.Current();
    lcdIndexes[lcdX * 2 + 1] = (byte)PortalRightAnimation.Current();
  }
  else
  {
    LcdCacheCreateChar(x, y, lcdIndexes, chars, lcdX, &maxCharIndex);
    ReconfigureFirstHeroChar(lcdIndexes, chars, lcdX, &maxCharIndex);
  }

  unsigned char newX = x;
  while (CheckLeftRoomAvailable(newX--, y) && (newX % LcdRenderW) < lcdX)
  {
    uint8_t nLcdX = newX % LcdRenderW;

    if (newX == Portal.X && y == Portal.Y)
    {
      lcdIndexes[nLcdX * 2] = (byte)PortalLeftAnimation.Current();
      lcdIndexes[nLcdX * 2 + 1] = (byte)PortalRightAnimation.Current();
    }
    else
    {
      LcdCacheCreateChar(newX, y, lcdIndexes, chars, nLcdX, &maxCharIndex);
    }
  }

  newX = x;
  while (CheckRightRoomAvailable(newX++, y) && (newX % LcdRenderW) > lcdX)
  {
    uint8_t nLcdX = newX % LcdRenderW;

    if (newX == Portal.X && y == Portal.Y)
    {
      lcdIndexes[nLcdX * 2] = (byte)PortalLeftAnimation.Current();
      lcdIndexes[nLcdX * 2 + 1] = (byte)PortalRightAnimation.Current();
    }
    else
    {
      LcdCacheCreateChar(newX, y, lcdIndexes, chars, nLcdX, &maxCharIndex);
    }
  }

  byte lcd2RowIndexes[2] = { 255, 255 };
  if (lcdY == 0)
  {
    if (CheckBottomRoomAvailable(x, y))
    {
      unsigned char incY = y + 1;

      if (x == Portal.X && incY == Portal.Y)
      {
        lcd2RowIndexes[0] = (byte)PortalLeftAnimation.Current();
        lcd2RowIndexes[1] = (byte)PortalRightAnimation.Current();
      }
      else
      {
        LcdCacheCreateChar(x, incY, lcd2RowIndexes, chars, 0, &maxCharIndex);
      }
    }
  }
  else
  {
    if (CheckTopRoomAvailable(x, y))
    {
      unsigned char decY = y - 1;

      if (x == Portal.X && decY == Portal.Y)
      {
        lcd2RowIndexes[0] = (byte)PortalLeftAnimation.Current();
        lcd2RowIndexes[1] = (byte)PortalRightAnimation.Current();
      }
      else
      {
        LcdCacheCreateChar(x, decY, lcd2RowIndexes, chars, 0, &maxCharIndex);
      }
    }      
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

  uint8_t otherLcdY = 0;
  if (lcdY == 0)
    otherLcdY = 1;

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
  if (lcd2RowIndexes[0] < 255)
  {    
    Lcd.write(lcd2RowIndexes[0]);
    Lcd.write(lcd2RowIndexes[1]);
  }
  else
  {
    Lcd.write((byte)LcdFogChar);
    Lcd.write((byte)LcdFogChar);
  }
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
bool InvokeGameLogicWorkerFlag = false;
bool InvokeGameRenderWorkerFlag = false;


void InputWorkerClbk(bool eventExec)
{
  LastAxisDiraction = GetAxisDiraction();

  if (LastAxisDiraction != ZeroDiraction)
    InvokeGameLogicWorkerFlag = true;
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


void GameLogicWorkerClbk(bool eventExec)
{
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
    InvokeGameRenderWorkerFlag = true;
  }
}
TimeWorker GameLogicWorker = TimeWorker(GameLoopDelay, GameLogicWorkerClbk, &InvokeGameLogicWorkerFlag);


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
}
TimeWorker GlobalAnimationWorker = TimeWorker(GlobalAnimationDelay, GlobalAnimationWorkerClbk);
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
