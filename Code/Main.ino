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

  
};
