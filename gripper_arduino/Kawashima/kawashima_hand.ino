int analogPin = 0;
int ledPin = 13;
double power = 30.0; //[N]
double g = 9.8;  //重力加速度

int val[7];
int threshold;
int i,j;

void  Swap(int *x, int *y)
{
  int work;        /* 作業用 */
  work = *x;        /* xの値を一時保存 */
  *x = *y;          /* xの値をyの値に書き換える */
  *y = work;        /* yの値をxの値に書き換える */
}

void Bubblesort(int a[]){
  for(int i=0;i<7;i++){
    for(int j=6;j>i;j--){
      if(a[j]<a[j-1]){
        Swap(&a[j],&a[j-1]);
      }
    }
  }
}

//起動直後に1回だけ実行される関数
//※リセット後もここから実行される
void setup() {
  int bias;

  Serial.begin(9600);        // シリアル通信の初期化
  pinMode(ledPin, OUTPUT);

  for(i=0;i<7;i++){
    delay(100);
    val[i] = analogRead(analogPin);
  }
  Bubblesort(val);
  int temp = val[3];

  for(i=0;i<7;i++){
    delay(100);
    val[i] = analogRead(analogPin);
  }
  Bubblesort(val);

  bias = (val[3]+temp)/2;

  Serial.print("bias = ");
  Serial.println(bias);

  Serial.print("target power = ");
  Serial.println(power);

  threshold = (bias - (int)((power/g)*30.0));
  //この値を下回るとスイッチをいれる

  Serial.print("threshold = ");
  Serial.println(threshold);

  delay(1000);
  delay(1000);
}

//setup()が実行された直後に繰り返し実行される関数
void loop() {

  delay(200); //メインループの待機時間

  // センサの値を5回読み取る
  for(i=0;i<7;i++){
    val[i] = analogRead(analogPin);
  }

  //中央値をとる
  Bubblesort(val);
  Serial.println(val[3]);

  if(val[3] < threshold){  //thresholdをセンサ値が下回ると
    digitalWrite(ledPin, HIGH);  //スイッチON
    delay(2000);
    
    while(1){
      for(i=0;i<7;i++){
        val[i] = analogRead(analogPin);
      }
      Bubblesort(val);

      Serial.print(val[3]);
      Serial.println("   now inhaling...");

      if(val[3] < threshold){
        digitalWrite(ledPin, LOW);
        delay(500);
        break;
      }
      delay(500);
    }
  }else{
    digitalWrite(ledPin, LOW);
  }
}
