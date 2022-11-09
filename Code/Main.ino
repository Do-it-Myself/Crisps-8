class LineFollower{
  private:
  static const threshold = 4;
  int inputPin;
  public:
  LineFollower (int p) {
    inputPin = p;
    pinMode(p, INPUT);
  }
  bool getLineData(){
    analog = digitalRead(inputPin)
    if (analog > threshold){
      return true;
    }else{
      return false
    }
  }
}

class Crisps{
  private:
  LineFollower lineFollowers[4];

  public:
  Crisps (int line1, int line2, int line3, int line4, int motor1, int motor2){
    lineFollowers[0] = LineFollower(line1);
    lineFollowers[1] = LineFollower(line1);
    lineFollowers[2] = LineFollower(line1);
    lineFollowers[3] = LineFollower(line1);

  boolean* getLineData(){
    static boolean r[4];
    for(int i = 0; i<4; i++){
      r[i] = lineFollowers[i].getLineData();
    }
    return r;
  }
  }

}
