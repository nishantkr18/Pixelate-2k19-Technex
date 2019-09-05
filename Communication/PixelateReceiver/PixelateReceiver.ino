void setup() 
{
  Serial.begin(9600);
  pinMode(3,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
}
int lt=0, rt=0;
int L = 0, R = 0;
void loop()
{ 
  if(Serial.available()>0)
  {
    lt = Serial.parseInt();
    rt = Serial.parseInt();
    L = Serial.parseInt();
    R = Serial.parseInt();
    char junk=Serial.read();
  }
  if(L==0){
  digitalWrite(7, LOW);
  digitalWrite(6, HIGH);
  }
  else
  {
    digitalWrite(7, HIGH);
  digitalWrite(6,LOW);
  }
  if(R==0){
  digitalWrite(9, LOW);
  digitalWrite(8, HIGH);
  }
  else
  {
    digitalWrite(9, HIGH);
  digitalWrite(8, LOW);
  }
  analogWrite(3, lt);
  analogWrite(10, rt);
Serial.print(lt);
Serial.print(' ');
Serial.print(rt);
Serial.print(' ');
Serial.print(L);
Serial.print(' ');
Serial.println(R);


  
  
}
