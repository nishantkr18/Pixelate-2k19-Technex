void setup() 
{
  pinMode(6, OUTPUT);
  pinMode(A5, OUTPUT);
  Serial.begin(9600);
}
int lt=0, rt=0, a=0, b=0, c=0;
void loop()
{ 
  if(Serial.available()>0)
  {
    lt = Serial.parseInt();
    rt = Serial.parseInt();
    a = Serial.parseInt();
    b = Serial.parseInt();
    c= Serial.parseInt();
    char junk=Serial.read();
      Serial.print(lt);
  Serial.print(' ');
  Serial.print(rt);
  Serial.print(' ');
  Serial.print(a);
  Serial.print(' ');
  Serial.print(b);
  Serial.print('\n');
    
  }

  if(lt==0 && rt==0)
  {
    digitalWrite(6, LOW);
  }
  else
  {
    digitalWrite(6, HIGH);
  }
//
  if(c==1)
  {
    digitalWrite(A5, HIGH);
    
  }
  
  
  
  
}
