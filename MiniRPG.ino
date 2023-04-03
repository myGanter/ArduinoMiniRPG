#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <LiquidCrystal.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


#define DEBUG_MOD false //set to true to duplicate the full status in the serial port
#define PRINT_LVL_TEXT true

#define W 10
#define H 10
#define LcdW 20
#define LcdH 4
#define LcdRenderW 10 // LcdW / 2
#define LcdRenderH 2
#define ActivateRoomValue 1
#define AxisThreshold 25 // % 
#define MinMaxAxisValues 14000 // max for GY-521 ~ 20000
#define StartMinMaxAxisValues 10000

#define StartLvl 1
#define StartSpawnLvlEnemy 3
#define StartBombCounter 3

#define InputDelay 10
#define HeroMoveDelay 300
#define EnemyMoveDelay 3000
#define LcdTextPrintDelay 300
#define GameRenderDelay 200 //5 fps
#define GlobalAnimationDelay 500
#define BombExplosionDelay 3000
#define BombExplosionSpreadDelay 500
#define MinimumBombExplosionActionTime 4000

#define LcdFogChar '#' //optionally 255 can be used
#define BombItemChar '@'
#define PortalLeftChars "({"
#define PortalRightChars ")}"
#define BombChars "@ "
#define ExplosionChars "-+*O*O*O*O*O*O"

#define EnemyAnimationsCount 3
//■■■■■
//■ ■ ■
//■■■■■
//■   ■
//■   ■
#define Enemy1Chars new char[1] { (char)252 }
// ■■■ 
//■   ■
//■   ■
// ■ ■ 
//■■ ■■
#define Enemy2Chars new char[1] { (char)244 }
// ■ ■    ■■■ 
//       ■   ■
// ■■■   ■   ■
//■   ■  ■   ■
//■   ■  ■   ■
//■   ■  ■   ■
// ■■■    ■■■ 
#define Enemy3Chars new char[2] { (char)239, (char)79 }


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

struct Note
{
  int Value;
  int Duration;
};

struct Sound
{
  Note *Notes;
  int Length;
  int CurrentPlayPos;
  unsigned long LastExecTime;
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

struct Enemy
{
  Point Position;
  bool PosIsRightChar;
  byte AnimationContainerIndex;
  Diraction LastDiraction;
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
    const char *f = first, *l = second;

    while (*f++) ++l1;
    while (*l++) ++l2;

    char *result = new char[l1 + l2 + 1];

    for (int i = 0; i < l1; i++) result[i] = first[i];
    for (int i = l1; i < l1 + l2; i++) result[i] = second[i - l1];

    result[l1 + l2] = '\0';
    return result;
}

int GetStrLen(const char *str)
{
  int res = 0;
  const char *f = str;

  while (*f++) ++res;

  return res;
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

    T* GetReferenceValue()
    {
      return &_currentNode->Value;
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

    void Delete(T val)
    {
      if (_count == 0)
        return;

      Node<T>* node = _currentNode;

      if (memcmp(&node->Value, &val, sizeof(T)) == 0)
      {
        _currentNode = node->NextNode;

        delete node;
        _count--;
        return;
      }
      
      while (node->NextNode != NULL)
      {
        if (memcmp(&node->NextNode->Value, &val, sizeof(T)) == 0)
        {
          Node<T>* delNode = node->NextNode;
          node->NextNode = delNode->NextNode;

          delete delNode;
          _count--;
          return;
        }

        node = node->NextNode;
      }
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
LiquidCrystal Lcd(11, 12, 2, 3, 4, 5, 6, 7, 8, 9);
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
  if (MaxAxisX > MinMaxAxisValues) MaxAxisX = MinMaxAxisValues;
  if (ax < MinAxisX) MinAxisX = ax;
  if (MinAxisX < -MinMaxAxisValues) MinAxisX = -MinMaxAxisValues;
  if (ay > MaxAxisY) MaxAxisY = ay;
  if (MaxAxisY > MinMaxAxisValues) MaxAxisY = MinMaxAxisValues;
  if (ay < MinAxisY) MinAxisY = ay;
  if (MinAxisY < -MinMaxAxisValues) MinAxisY = -MinMaxAxisValues;
  
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
bool IsHeroDied = false;
bool IsHeroLastSound1 = false;
Stack<Enemy> Enemies = Stack<Enemy>();
Point Portal = { .X = 0, .Y = 0 };
Diraction LastAxisDiraction = ZeroDiraction;
AppState CurrentAppState = PrintInfo;
unsigned int LvlCounter = StartLvl;
ButtonValue Button = NoneButton;
int LastEnemiesUiStringLen = -1;
int LastBombCounterUiStringLen = -1;

//--BOMB--
byte BombCounter = StartBombCounter;
Point *BombItem = NULL;
Point *Bomb = NULL;
bool BombIsRight = false;
bool BombIsExplosion = false;
Stack<Point> *ExplosionPoints = NULL;
Point ExplosionTop = { .X = 0, .Y = 0 };
Point ExplosionRight = { .X = 0, .Y = 0 };
Point ExplosionLeft = { .X = 0, .Y = 0 };
Point ExplosionBottom = { .X = 0, .Y = 0 };
//-------------------- end global game vars


//-------------------- global infrastructure vars
bool InvokeHeroMoveWorkerFlag = false;
bool InvokeGameRenderWorkerFlag = false;
bool InvokeHeroBombWorkerFlag = false;
bool BombExplosionWorkerFlag = false;
bool GlobalSoundWorkerFlag = false;
byte BombExplosionWorkerCounter = 0;

char *LcdText = NULL;
int TextIndex = 0;
int TextOffSet = 0;
Sound *SoundChannel = NULL;
//-------------------- end global infrastructure vars


//-------------------- map
void GenerateMap()
{
#if W > 11 || H > 10
  GenerateMapStupidAlg();
#else
  GenerateMapWormAlg();
#endif
}

void GenerateMapStupidAlg()
{
  for (int x = 0; x < W - 1; ++x)
  {
    RoomAddValue({ .X = x, .Y = 0 }, Right);        
  }

  for (int y = 0; y < H; ++y)
  {
    int bucketCounter = random(W / 3) + 2;
    int bucketLen = W / bucketCounter;
    int mod = (int)ceil((float)bucketCounter / (W % bucketCounter));
    int modLen = W % bucketCounter != 0 ? (W - bucketCounter * bucketLen) / (bucketCounter / mod) : 0;

    int fixer = 0;

    for (int i = 0; i < bucketCounter; ++i)
    {
      if (W % bucketCounter != 0 && (i + 1) % mod == 0)
      {
        int max = i * bucketLen + fixer + modLen;

        for (int modx = i * bucketLen + fixer; modx < max; ++modx)
        {
          if (y - 1 > -1)
            RoomAddValue({ .X = modx, .Y = y - 1 }, Down);

          fixer++;
        }
      }

      int curx = i * bucketLen + fixer;

      for (int bucketx = curx; bucketx < curx + bucketLen - 1; ++bucketx)
      {
        RoomAddValue({ .X = bucketx, .Y = y }, Right);
      }

      if (y - 1 > -1)
        RoomAddValue({ .X = random(bucketLen) + curx, .Y = y - 1 }, Down);        
    }

    for (int x = bucketCounter / mod * modLen + bucketLen * bucketCounter; x < W; ++x)
    {
      if (y - 1 > -1)
        RoomAddValue({ .X = x, .Y = y - 1 }, Down);
    }

    for (int x = 0; x < W; ++x)
    {
      if (y < H - 1 && random(100) < 5)
        RoomAddValue({ .X = x, .Y = y }, Down);
      
      if (x < W - 1 && random(100) < 5)
        RoomAddValue({ .X = x, .Y = y }, Right);
    }
  }
}

void GenerateMapWormAlg()
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
      else if (BombItem != NULL && BombItem->Y == y && BombItem->X == x)
      {
        Serial.print("@ ");
        offset = "";
      }
      else if (Enemies.Count() > 0)
      {
        Enemy *enemy = GetEnemyFromPoint(x, y);
        if (enemy != NULL)
        {
          if (enemy->PosIsRightChar)
            Serial.print(" ?");
          else
            Serial.print("? ");
          offset = "";
        }
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
  Serial.print("  Portal "); Serial.print(Portal.X); Serial.print(":"); Serial.print(Portal.Y);
  if (BombItem != NULL)
  {
    Serial.print("  Bomb Item "); Serial.print(BombItem->X); Serial.print(":"); Serial.println(BombItem->Y);
  }
  else
  {
    Serial.println();
  }
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

Point GenerateRandomPoint()
{
  Point res;

  res.X = random(W);
  res.Y = random((res.X >= (W / 2) ? 0 : H / 2), H);

  return res;
}

void IninEnamies()
{
  Enemies.Clear();
  int maxEnemy = sqrt(W * H) / 2;
  int enemyCount = LvlCounter - StartSpawnLvlEnemy;
  if (enemyCount > 0)
  {
    if (enemyCount > maxEnemy)
      enemyCount = maxEnemy;

    for (int i = 0; i < enemyCount; ++i)
    {
      Enemy enemy = 
      { 
        .Position = GenerateRandomPoint(), 
        .PosIsRightChar = false, 
        .AnimationContainerIndex = random(EnemyAnimationsCount), 
        .LastDiraction = ZeroDiraction 
      };

      Enemies.Push(enemy);
    }
  }
}

void InitOtherPoint()
{ 
  Portal = GenerateRandomPoint();

  Point bombItemPoint = GenerateRandomPoint();
  BombItem = new Point();
  BombItem->X = bombItemPoint.X;
  BombItem->Y = bombItemPoint.Y;

  IninEnamies();
}

void ClearBombItem()
{
  if (BombItem != NULL)
  {
    Point *currentItemBomb = BombItem;
    BombItem = NULL;
    delete currentItemBomb;
  } 
}

void ClearBomb()
{
  if (Bomb == NULL)
    return;

  BombIsExplosion = false;

  Point *currentBomb = Bomb;
  Bomb = NULL;
  delete currentBomb;  

  Stack<Point> *currentExplosionPoints = ExplosionPoints;
  ExplosionPoints->Clear();
  ExplosionPoints = NULL;
  delete currentExplosionPoints;
}
//-------------------- end map


//-------------------- hero
bool PointMoveTo(Diraction diraction, Point *point, bool *pointPosIsRightChar)
{
  if (diraction == Top)
  {
    if ((point->Y - 1) > -1 && RoomIsDown(Map[point->Y - 1][point->X]))
    {
      point->Y -= 1;
      return true;
    }  
  }
  else if (diraction == RightDiraction)
  {
    if (!(*pointPosIsRightChar))
    {
      (*pointPosIsRightChar) = true;
      return true;
    }
    else if ((point->X + 1) < W && RoomIsRight(Map[point->Y][point->X]))
    {
      point->X += 1;
      (*pointPosIsRightChar) = false;
      return true;
    }
  }
  else if (diraction == Bottom)
  {
    if ((point->Y + 1) < H && RoomIsDown(Map[point->Y][point->X]))
    {
      point->Y += 1;
      return true;
    }
  }
  else if (diraction == Left)
  {
    if ((*pointPosIsRightChar))
    {
      (*pointPosIsRightChar) = false;
      return true;
    }
    else if ((point->X - 1) > -1 && RoomIsRight(Map[point->Y][point->X - 1]))
    {
      point->X -= 1;
      (*pointPosIsRightChar) = true;
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

Diraction GetXDiractionFromInt(int value)
{
  if (value < 0)
    return Left;
  else if (value > 0)
    return RightDiraction;
  else
    return ZeroDiraction;
}

Diraction GetYDiractionFromInt(int value)
{
  if (value < 0)
    return Top;
  else if (value > 0)
    return Bottom;
  else
    return ZeroDiraction;
}

/*void EnemyAIAlg1(Enemy *enemy)
{
  Point pastPoint = enemy->Position;
  bool pastRightPos = enemy->PosIsRightChar;

  int dx = (int)Hero.X - enemy->Position.X;
  int dy = (int)Hero.Y - enemy->Position.Y;
  
  if (dx == 0 && dy == 0)
  {
    enemy->PosIsRightChar = HeroPosIsRightChar;
    return;
  }

  if (abs(dx) > abs(dy))
  {
    Diraction xDiraction = GetXDiractionFromInt(dx);

    if (!PointMoveTo(xDiraction, &enemy->Position, &enemy->PosIsRightChar))
    {       
      Diraction yDiraction = GetYDiractionFromInt(dy);

      if (!PointMoveTo(yDiraction, &enemy->Position, &enemy->PosIsRightChar))
      {
        if (!PointMoveTo(yDiraction == Top ? Bottom : Top, &enemy->Position, &enemy->PosIsRightChar))
        {
          PointMoveTo(xDiraction == RightDiraction ? Left : RightDiraction, &enemy->Position, &enemy->PosIsRightChar);
        }
      }
    }
  }
  else
  {
    Diraction yDiraction = GetYDiractionFromInt(dy);

    if (!PointMoveTo(yDiraction, &enemy->Position, &enemy->PosIsRightChar))
    {
      Diraction xDiraction = GetXDiractionFromInt(dx);

      if (!PointMoveTo(xDiraction, &enemy->Position, &enemy->PosIsRightChar))
      {
        if (!PointMoveTo(yDiraction == Top ? Bottom : Top, &enemy->Position, &enemy->PosIsRightChar))
        {
          PointMoveTo(xDiraction == RightDiraction ? Left : RightDiraction, &enemy->Position, &enemy->PosIsRightChar);
        }
      }
    }
  }

  if (GetCountEnemiesFromPoint(enemy->Position.X, enemy->Position.Y) > 1)
  {
    enemy->Position = pastPoint;
    enemy->PosIsRightChar = pastRightPos;
  }
}*/

void EnemyAIAlg2(Enemy *enemy)
{
  Point pastPoint = enemy->Position;
  bool pastRightPos = enemy->PosIsRightChar;
  int countEnemiesFromPoint = 0;

  if (Hero.X == pastPoint.X && Hero.Y == pastPoint.Y)
  {
    enemy->PosIsRightChar = HeroPosIsRightChar;
    return;
  }

  Point visionZoneLine = enemy->Position;
  if (enemy->Position.X == Hero.X)
  {
    while (CheckTopRoomAvailable(visionZoneLine.X, visionZoneLine.Y))
    {
      visionZoneLine.Y--;

      if (Hero.Y == visionZoneLine.Y)
      {
        enemy->LastDiraction = Top;
        break;
      }
    }
    
    visionZoneLine = enemy->Position;
    while (CheckBottomRoomAvailable(visionZoneLine.X, visionZoneLine.Y))
    {
      visionZoneLine.Y++;

      if (Hero.Y == visionZoneLine.Y)
      {
        enemy->LastDiraction = Bottom;
        break;
      }
    }
  }

  if (enemy->Position.Y == Hero.Y)
  {
    while (CheckLeftRoomAvailable(visionZoneLine.X, visionZoneLine.Y))
    {
      visionZoneLine.X--;

      if (Hero.X == visionZoneLine.X)
      {
        enemy->LastDiraction = Left;
        break;
      }
    }
    
    visionZoneLine = enemy->Position;
    while (CheckRightRoomAvailable(visionZoneLine.X, visionZoneLine.Y))
    {
      visionZoneLine.X++;

      if (Hero.X == visionZoneLine.X)
      {
        enemy->LastDiraction = RightDiraction;
        break;
      }
    }
  }

  if (!PointMoveTo(enemy->LastDiraction, &enemy->Position, &enemy->PosIsRightChar) || 
    (countEnemiesFromPoint = GetCountEnemiesFromPoint(enemy->Position.X, enemy->Position.Y)) > 1)
  {
    if (countEnemiesFromPoint > 1)
    {
      enemy->Position = pastPoint;
      enemy->PosIsRightChar = pastRightPos;
    }

    Diraction diractions[4] = { ZeroDiraction, ZeroDiraction, ZeroDiraction, ZeroDiraction };
    int diractionsIndexes = 0;

    if (enemy->LastDiraction != Top && CheckTopRoomAvailable(enemy->Position.X, enemy->Position.Y))
    {
      diractions[diractionsIndexes++] = Top;
    }
    
    if (enemy->LastDiraction != RightDiraction && CheckRightRoomAvailable(enemy->Position.X, enemy->Position.Y))
    {
      diractions[diractionsIndexes++] = RightDiraction;
    }

    if (enemy->LastDiraction != Bottom && CheckBottomRoomAvailable(enemy->Position.X, enemy->Position.Y))
    {
      diractions[diractionsIndexes++] = Bottom;
    }

    if (enemy->LastDiraction != Left && CheckLeftRoomAvailable(enemy->Position.X, enemy->Position.Y))
    {
      diractions[diractionsIndexes++] = Left;
    }

    if (diractionsIndexes > 1)
    {
      Diraction searchDiraction = ZeroDiraction;
      
      if (enemy->LastDiraction == Top)
        searchDiraction = Bottom;
      else if (enemy->LastDiraction == Bottom)
        searchDiraction = Top;
      else if (enemy->LastDiraction == Left)
        searchDiraction = RightDiraction;
      else if (enemy->LastDiraction == RightDiraction)
        searchDiraction = Left;

      for (int i = 0; i < diractionsIndexes; ++i)
      {
        if (diractions[i] == searchDiraction)
        {
          int replaseIndex = (i + 1) % diractionsIndexes;
          diractions[i] = diractions[replaseIndex];

          break;
        }
      }
    }

    enemy->LastDiraction = diractions[random(diractionsIndexes)];
  }
  else
  {
    Diraction diractions[2] = { ZeroDiraction, ZeroDiraction };
    int diractionsIndexes = 0;

    if ((enemy->LastDiraction == Left || enemy->LastDiraction == RightDiraction) && random(100) > 60)
    {
      if (CheckTopRoomAvailable(enemy->Position.X, enemy->Position.Y))
      {
        diractions[diractionsIndexes++] = Top;
      }

      if (CheckBottomRoomAvailable(enemy->Position.X, enemy->Position.Y))
      {
        diractions[diractionsIndexes++] = Bottom;
      }
    }
    else if ((enemy->LastDiraction == Top || enemy->LastDiraction == Bottom) && random(100) > 60)
    {
      if (CheckRightRoomAvailable(enemy->Position.X, enemy->Position.Y))
      {
        diractions[diractionsIndexes++] = RightDiraction;
      }

      if (CheckLeftRoomAvailable(enemy->Position.X, enemy->Position.Y))
      {
        diractions[diractionsIndexes++] = Left;
      }
    }

    if (diractionsIndexes > 0)
      enemy->LastDiraction = diractions[random(diractionsIndexes)];
  }
}

void CheckHeroDie()
{
  if (Enemies.Count() > 0)
  {
    Enemy *enemy = GetEnemyFromPoint(Hero.X, Hero.Y);

    if (enemy != NULL && enemy->PosIsRightChar == HeroPosIsRightChar)
    {
      IsHeroDied = true;
    }
  }
}
//-------------------- end hero


//-------------------- sound
const Note WalkSound1[1] PROGMEM = { { .Value = 300, .Duration = 200 } };
const Note WalkSound2[1] PROGMEM = { { .Value = 400, .Duration = 200 } };
const Note TextSound[1] PROGMEM = { { .Value = 1000, .Duration = 70 } };
const Note DieSound[31] PROGMEM = { { .Value = 466, .Duration = 1430 }, { .Value = 349, .Duration = 237 }, { .Value = 349, .Duration = 237 }, { .Value = 466, .Duration = 237 }, { .Value = 415, .Duration = 118 }, { .Value = 370, .Duration = 118 }, { .Value = 415, .Duration = 1430 }, { .Value = 466, .Duration = 1430 }, { .Value = 370, .Duration = 237 }, { .Value = 370, .Duration = 237 }, { .Value = 466, .Duration = 237 }, { .Value = 440, .Duration = 118 }, { .Value = 392, .Duration = 118 }, { .Value = 440, .Duration = 1430 }, { .Value = 466, .Duration = 476 }, { .Value = 349, .Duration = 714 }, { .Value = 466, .Duration = 237 }, { .Value = 466, .Duration = 118 }, { .Value = 523, .Duration = 118 }, { .Value = 587, .Duration = 118 }, { .Value = 622, .Duration = 118 }, { .Value = 698, .Duration = 954 }, { .Value = 698, .Duration = 237 }, { .Value = 698, .Duration = 237 }, { .Value = 698, .Duration = 237 }, { .Value = 740, .Duration = 118 }, { .Value = 831, .Duration = 118 }, { .Value = 932, .Duration = 1430 }, { .Value = 1109, .Duration = 476 }, { .Value = 1047, .Duration = 476 }, { .Value = 880, .Duration = 954 } };
const Note WinSound[128] PROGMEM = { { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 117, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 123, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 117, .Duration = 799 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 117, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 123, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 185, .Duration = 99 }, { .Value = 147, .Duration = 99 }, { .Value = 123, .Duration = 99 }, { .Value = 220, .Duration = 99 }, { .Value = 185, .Duration = 99 }, { .Value = 123, .Duration = 99 }, { .Value = 147, .Duration = 99 }, { .Value = 185, .Duration = 99 }, { .Value = 220, .Duration = 99 }, { .Value = 185, .Duration = 99 }, { .Value = 147, .Duration = 99 }, { .Value = 123, .Duration = 99 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 117, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 123, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 117, .Duration = 799 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 117, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 123, .Duration = 133 }, { .Value = 131, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 165, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 147, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 82, .Duration = 133 }, { .Value = 247, .Duration = 99 }, { .Value = 196, .Duration = 99 }, { .Value = 165, .Duration = 99 }, { .Value = 196, .Duration = 99 }, { .Value = 247, .Duration = 99 }, { .Value = 330, .Duration = 99 }, { .Value = 196, .Duration = 99 }, { .Value = 247, .Duration = 99 }, { .Value = 330, .Duration = 99 }, { .Value = 247, .Duration = 99 }, { .Value = 392, .Duration = 99 }, { .Value = 494, .Duration = 99 } };
const Note BombTakeSound[4] PROGMEM = { { .Value = 1300, .Duration = 70 }, { .Value = 1000, .Duration = 70 }, { .Value = 1300, .Duration = 70 }, { .Value = 1000, .Duration = 250 } };
const Note BombExplosionSound[16] PROGMEM = { { .Value = 400, .Duration = 70 }, { .Value = 370, .Duration = 70 }, { .Value = 430, .Duration = 70 }, { .Value = 400, .Duration = 70 }, { .Value = 370, .Duration = 70 }, { .Value = 430, .Duration = 70 }, { .Value = 400, .Duration = 70 }, { .Value = 370, .Duration = 70 }, { .Value = 430, .Duration = 70 }, { .Value = 400, .Duration = 70 }, { .Value = 370, .Duration = 70 }, { .Value = 430, .Duration = 70 }, { .Value = 400, .Duration = 70 }, { .Value = 370, .Duration = 70 }, { .Value = 430, .Duration = 70 }, { .Value = 433, .Duration = 120 } };


void ClearSoundChannel()
{
  Sound *currentSound = SoundChannel;

  noTone(13);

  currentSound->Notes = NULL;
  SoundChannel = NULL;
   
  delete currentSound; 
}

void PlaySound(Note *notes, int length, bool reWrite = true)
{
  if (SoundChannel != NULL)
  {
    if (reWrite)
      ClearSoundChannel();
    else
      return;
  }

  SoundChannel = new Sound();
  SoundChannel->Length = length;
  SoundChannel->Notes = notes;

  GlobalSoundWorkerFlag = true;
}
//-------------------- end sound


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
AnimationContainer<char> EnemyAnimations[EnemyAnimationsCount] = 
{
  AnimationContainer<char>(Enemy1Chars, 1, sizeof(char), GlobalAnimationDelay),
  AnimationContainer<char>(Enemy2Chars, 1, sizeof(char), GlobalAnimationDelay),
  AnimationContainer<char>(Enemy3Chars, 2, sizeof(char), GlobalAnimationDelay)
};

void InitLcd()
{
  Lcd.begin(LcdW, LcdH);
  Lcd.clear();
  Lcd.noCursor();
  Lcd.noBlink();
  Lcd.noAutoscroll();
}

void MergeChars(byte target[], byte source[])
{
  for (int i = 0; i < 8; ++i)
    target[i] |= source[i];
}

bool CreateRoomLeftChar(byte target[], Diraction diraction)
{
  bool noEmptyResult = false;

  if ((diraction & Top) == Top)
  {
    MergeChars(target, TopWall);
    noEmptyResult = true;
  }

  if ((diraction & Bottom) == Bottom)
  {
    MergeChars(target, BottomWall);
    noEmptyResult = true;
  }

  if ((diraction & Left) == Left)
  {
    MergeChars(target, LeftWall);
    noEmptyResult = true;
  }

  return noEmptyResult;
}

bool CreateRoomRightChar(byte target[], Diraction diraction)
{
  bool noEmptyResult = false;

  if ((diraction & Top) == Top)
  {
    MergeChars(target, TopWall);
    noEmptyResult = true;
  }

  if ((diraction & Bottom) == Bottom)
  {
    MergeChars(target, BottomWall);
    noEmptyResult = true;
  }

  if ((diraction & RightDiraction) == RightDiraction)
  {
    MergeChars(target, RightWall);
    noEmptyResult = true;
  }

  return noEmptyResult;
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
  if (currentPos == ZeroDiraction)
  {
    byte spaceChar = (byte)' ';

    if ((charExist & Left) != Left)
      lcdIndexes[lcdX * 2] = spaceChar;

    if ((charExist & RightDiraction) != RightDiraction)
      lcdIndexes[lcdX * 2 + 1] = spaceChar;

    return;
  }

  byte findIndex = 0;
  if (!CreateRoomLeftChar(chars[*maxCharIndex], currentPos))
  {
    if ((charExist & Left) != Left)
      lcdIndexes[lcdX * 2] = (byte)' ';
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

    if ((charExist & Left) != Left)
    {
      lcdIndexes[lcdX * 2] = findIndex;
    }
  }  
  
  if (!CreateRoomRightChar(chars[*maxCharIndex], currentPos))
  {
    if ((charExist & RightDiraction) != RightDiraction)
      lcdIndexes[lcdX * 2 + 1] = (byte)' ';
  }
  else
  {   
    if ((*maxCharIndex) > 0 && EqualChars(chars[*maxCharIndex], chars[findIndex]))
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
}

void ReconfigureFirstHeroChar(byte lcdIndexes[], byte chars[][8], uint8_t lcdX, byte *maxCharIndex)
{
  byte spaceChar = (byte)' ';
  int lcdIndex = lcdX * 2;
  bool rightFree = lcdIndexes[lcdIndex + 1] < 8;
  bool heroFree = lcdIndexes[lcdIndex + (int)HeroPosIsRightChar] < 8 || lcdIndexes[lcdIndex + (int)HeroPosIsRightChar] == spaceChar;
  bool leftSpace = lcdIndexes[lcdIndex] == spaceChar;
  bool rightSpace = lcdIndexes[lcdIndex + 1] == spaceChar;

  if ((*maxCharIndex) < 1)
  {
    (*maxCharIndex)++;

    if (heroFree)
      lcdIndexes[lcdIndex + (int)HeroPosIsRightChar] = 0;

    MergeChars(chars[0], HeroAnimation.Current());
  }
  else if ((*maxCharIndex) < 2)
  {
    if (leftSpace || rightSpace)
    {
      if ((!HeroPosIsRightChar && rightSpace) || (HeroPosIsRightChar && leftSpace))
      {
        MergeChars(chars[0], HeroAnimation.Current());
      }
      else
      {
        (*maxCharIndex)++;
        MergeChars(chars[1], HeroAnimation.Current());
        lcdIndexes[lcdIndex + (int)HeroPosIsRightChar] = 1;
      }
    }
    else
    {
      (*maxCharIndex)++;
    
      MergeChars(chars[1], chars[0]);

      if (rightFree)
        lcdIndexes[lcdIndex + 1] = 1;

      MergeChars(chars[(int)HeroPosIsRightChar], HeroAnimation.Current());
    }
  }
  else
  {
    MergeChars(chars[(int)HeroPosIsRightChar], HeroAnimation.Current());
  }
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

int GetCountEnemiesFromPoint(unsigned char x, unsigned char y)
{
  int result = 0;
  NodeIterator<Enemy> *iterator = Enemies.CreateIterator();

  do
  {
    Enemy *enemy = iterator->GetReferenceValue();
    if (enemy->Position.X == x && enemy->Position.Y == y)
    {
      result++;
    }
  } 
  while (iterator->MoveNext());

  delete iterator;
  return result;
}

Enemy* GetEnemyFromPoint(unsigned char x, unsigned char y)
{
  NodeIterator<Enemy> *iterator = Enemies.CreateIterator();

  do
  {
    Enemy *enemy = iterator->GetReferenceValue();
    if (enemy->Position.X == x && enemy->Position.Y == y)
    {
      delete iterator;
      return enemy;
    }
  } 
  while (iterator->MoveNext());

  delete iterator;
  return NULL;
}

Diraction LcdCacheCreateCharCheckExistingGameObjs(unsigned char x, unsigned char y, byte lcdIndexes[], int leftIndex, int rightIndex)
{
  byte bombChar = (byte)*BombAnimation.Current();
  Enemy *enemy = NULL;

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
  else if (Enemies.Count() > 0 && (enemy = GetEnemyFromPoint(x, y)) != NULL)
  {
    byte enemyChar = (byte)*EnemyAnimations[enemy->AnimationContainerIndex].Current();
    lcdIndexes[enemy->PosIsRightChar ? rightIndex : leftIndex] = enemyChar;

    return enemy->PosIsRightChar ? RightDiraction : Left;
  }
  else if (BombItem != NULL && x == BombItem->X && y == BombItem->Y)
  {
    lcdIndexes[leftIndex] = BombItemChar;
    return Left;
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

int LcdDrawTextValueInfo(byte y, char *str, int value, int lastUiStringLen)
{
  String *valueCounterStr = new String(value);
  char *stringValueStrTemp = StringConcatenate(str, valueCounterStr->c_str());
  int stringValueStrLen = GetStrLen(stringValueStrTemp);
  if (lastUiStringLen != stringValueStrLen)
  {
    Lcd.setCursor(0, y);

    for (int i = 0; i < lastUiStringLen; ++i)
      Lcd.write((byte)' ');

    lastUiStringLen = stringValueStrLen;
    
    Lcd.setCursor(0, y);
    Lcd.print(stringValueStrTemp);
  }
  else
  {
    Lcd.setCursor(GetStrLen(str), y);
    Lcd.print(valueCounterStr->c_str());
  }

  delete valueCounterStr;
  delete stringValueStrTemp;

  return lastUiStringLen;
}

void LcdDrawUI()
{
  const char *enemiesStr = "Enemies: ";
  const char *bombsStr = "Bombs: ";

  LastEnemiesUiStringLen = LcdDrawTextValueInfo(2, enemiesStr, Enemies.Count(), LastEnemiesUiStringLen);
  LastBombCounterUiStringLen = LcdDrawTextValueInfo(3, bombsStr, BombCounter, LastBombCounterUiStringLen);  
}

void LcdDrawMap(unsigned char x, unsigned char y)
{
  uint8_t lcdY = y % LcdRenderH;
  uint8_t lcdX = x % LcdRenderW;

  byte lcdIndexes[LcdW];
  ClearArr<byte>(lcdIndexes, LcdW, 255);

  byte maxCharIndex = 0;
  byte chars[8][8];
  for (int i = 0; i < 8; ++i)
    ClearArr<byte>(chars[i], 8, 0);

  if (!LcdCacheCreateCharCheckExistingGameObjsOrDefault(x, y, lcdIndexes, chars, &maxCharIndex))
  {
    ReconfigureFirstHeroChar(lcdIndexes, chars, lcdX, &maxCharIndex);
  }

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

#if LcdH > 2
  LcdDrawUI();
#endif
}

void InitLcdNextLvlText()
{
  Lcd.clear();

  const char *depth = "DEPTH ";
  const char *goodluck = "\ngood luck...  ";
  String *lvlCounterStr = new String(LvlCounter);
  char *temp = StringConcatenate(depth, lvlCounterStr->c_str());
  
  LcdText = StringConcatenate(temp, goodluck);

  delete lvlCounterStr;
  delete temp;
}

void InitLcdGameOverText()
{
  Lcd.clear();

  const char *text = "Luck has turned\nits back on you";
  const char *dots = "....";
  
  LcdText = StringConcatenate(text, dots);
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

      if (LcdText[TextIndex] != ' ')
        PlaySound(TextSound, sizeof(TextSound) / sizeof(Note), false);

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
void InputWorkerClbk(bool eventExec)
{
  LastAxisDiraction = GetAxisDiraction();

  if (LastAxisDiraction != ZeroDiraction)
    InvokeHeroMoveWorkerFlag = true;

  int buttonValue = analogRead(1);
  if (buttonValue > 900)
    Button = SelectButton;    
  else
    Button = NoneButton;
  
  //todo
  //Button = TopButton;
  //Button = RightButton;
  //Button = BottomButton;
  //Button = LeftButton;
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
    PlaySound(WinSound, sizeof(WinSound) / sizeof(Note));
    SwichModeToPrintLcdText();
  }
  else if (IsHeroDied)
  {    
    SwichModeToPrintLcdText();
  }
  else if (!IsHeroDied && PointMoveTo(LastAxisDiraction, &Hero, &HeroPosIsRightChar))
  {
    if (Bomb != NULL && Hero.X == Bomb->X && Hero.Y == Bomb->Y && HeroPosIsRightChar == BombIsRight)
    {
      Hero = pastHero;
      HeroPosIsRightChar = pastHeroPosIsRightChar;
    }
    else
    {
      if (IsHeroLastSound1)
        PlaySound(WalkSound2, sizeof(WalkSound2) / sizeof(Note), false);
      else
        PlaySound(WalkSound1, sizeof(WalkSound1) / sizeof(Note), false);

      IsHeroLastSound1 = !IsHeroLastSound1;

      InvokeGameRenderWorkerFlag = true;

      if (BombItem != NULL && Hero.X == BombItem->X && Hero.Y == BombItem->Y && !HeroPosIsRightChar)
      {
        PlaySound(BombTakeSound, sizeof(BombTakeSound) / sizeof(Note));
        BombCounter++;
        ClearBombItem();
      }

      CheckHeroDie();
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
    ClearBomb();

    if (IsHeroDied)
    {
      SwichModeToPrintLcdText();
    }
  }
  else
  {
    BombExplosionWorkerCounter++;

    if (!IsHeroDied && ((Hero.X == Bomb->X && Hero.Y == Bomb->Y) || LcdCheckExplosionPoint(Hero.X, Hero.Y)))
    {
      IsHeroDied = true;
    }

    if (Enemies.Count() > 0)
    {
      NodeIterator<Enemy> *enemiesIterator = Enemies.CreateIterator();

      do
      {
        Enemy *enemy = enemiesIterator->GetReferenceValue();
        if (LcdCheckExplosionPoint(enemy->Position.X, enemy->Position.Y) || (enemy->Position.X == Bomb->X && enemy->Position.Y == Bomb->Y))
        {
          Enemies.Delete(*enemy);
          break;
        }
      } 
      while (enemiesIterator->MoveNext());

      delete enemiesIterator;
    }
  }
}
TimeWorker BombExplosionWorker = TimeWorker(BombExplosionSpreadDelay, BombExplosionWorkerClbk, &BombExplosionWorkerFlag, false);


void HeroBombWorkerClbk(bool eventExec)
{
  if (eventExec)
    return;

  if (!BombIsExplosion)
  {
    PlaySound(BombExplosionSound, sizeof(BombExplosionSound) / sizeof(Note));

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


void EnemyMoveWorkerClbk(bool eventExec)
{
  //InvokeGameRenderWorkerFlag = true;

  if (Enemies.Count() == 0)
    return;

  NodeIterator<Enemy> *enemiesIterator = Enemies.CreateIterator();
  do
  {
    Enemy *enemy = enemiesIterator->GetReferenceValue();    

    EnemyAIAlg2(enemy);    
  } 
  while (enemiesIterator->MoveNext());

  delete enemiesIterator;

  CheckHeroDie();
}
TimeWorker EnemyMoveWorker = TimeWorker(EnemyMoveDelay, EnemyMoveWorkerClbk);


void GameLogicWorkerClbk(bool eventExec)
{ 
  HeroMoveWorker.Update();
  EnemyMoveWorker.Update();

  if (Button == SelectButton && Bomb == NULL && BombCounter > 0)
  {
    BombCounter--;
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


void SwichModeToPrintLcdText()
{
  if (IsHeroDied)
  {
    PlaySound(DieSound, sizeof(DieSound) / sizeof(Note));
    InitLcdGameOverText();
  }
  else
  {
    InitLcdNextLvlText();
  }

  GameRenderWorker.SetOnlyEventInvoked(true);

  CurrentAppState = PrintInfo;
}
void LcdTextPrintWorkerClbk(bool eventExec)
{
#if PRINT_LVL_TEXT
  if (!DrawChar())
#else
  if (true)
#endif
  {
    char *ptr = LcdText;
    LcdText = NULL;
    delete ptr;
    TextIndex = 0;
    TextOffSet = 0;

    if (!IsHeroDied)
    {
      LvlCounter++;
    }
    else
    {
      IsHeroDied = false;
      LvlCounter = 1;
      BombCounter = StartBombCounter;
      SwichModeToPrintLcdText();
      return;
    }

    Hero.X = 0;
    Hero.Y = 0;    
    HeroPosIsRightChar = false;
    LastEnemiesUiStringLen = -1;
    LastBombCounterUiStringLen = -1;

    ClearMap();
    ClearBomb();
    ClearBombItem();
    GenerateMap();
    InitOtherPoint();
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
  
  for (int i = 0; i < EnemyAnimationsCount; ++i)
    EnemyAnimations[i].IncrementIndex();
}
TimeWorker GlobalAnimationWorker = TimeWorker(GameRenderDelay, GlobalAnimationWorkerClbk);


void GlobalSoundWorkerClbk(bool eventExec)
{
  if (SoundChannel == NULL)
    return;

  Sound *currentSound = SoundChannel;
  Note *notes = currentSound->Notes;
  Note currentNote;
  Note prevNote;

  memcpy_P(&currentNote, &notes[currentSound->CurrentPlayPos >= currentSound->Length ? currentSound->CurrentPlayPos - 1 : currentSound->CurrentPlayPos], sizeof(Note));
  memcpy_P(&prevNote, &notes[currentSound->CurrentPlayPos > 0 ? currentSound->CurrentPlayPos - 1 : 0], sizeof(Note));
  unsigned long curTime = millis();

  if (currentSound->CurrentPlayPos >= currentSound->Length && (curTime - currentSound->LastExecTime) >= currentNote.Duration)
  {
    ClearSoundChannel();
    return;
  }
  else if ((curTime - currentSound->LastExecTime) >= prevNote.Duration)
  {
    if (currentNote.Value == 0)
      noTone(13);
    else
      tone(13, currentNote.Value, currentNote.Duration);

    currentSound->CurrentPlayPos++;
    currentSound->LastExecTime = curTime;
  }  
}
TimeWorker GlobalSoundWorker = TimeWorker(InputDelay, GlobalSoundWorkerClbk, &GlobalSoundWorkerFlag, false);
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

  randomSeed(analogRead(0));

  InitLcd();

  pinMode(13, OUTPUT);

  if (!InitAxis())
  {
    ArduinoOff(); 
    return;
  }

  SwichModeToPrintLcdText();
}

void loop() 
{
  InputWorker.Update();
  GlobalSoundWorker.Update();

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
