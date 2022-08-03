/* 	Programado por Lucas Campos Achcar
	Para ser apresentado pela Elizabeth do Carmo Campos
	no curso de robótica nas escolas
*/

#include <Servo.h>

// input dos motores

#define JOYSTICK_X1 A0
#define JOYSTICK_Y1 A1

#define JOYSTICK_X2 A2
#define JOYSTICK_Y2 A3

// output dos motores

#define MOTOR_BASE 		5
#define MOTOR_DIREITO 	6
#define MOTOR_ESQUERDO	9
#define MOTOR_GARRA		3

// limites de ação do joystick

#define LIMIT_RIGHT 800
#define LIMIT_LEFT 200

#define LIMIT_UP 800
#define LIMIT_DOWN 200

/////////////////////////////////

enum E_JOYSTICK 
{
  NONE,
  LEFT,
  RIGHT,
  UP,
  DOWN
};

class Joystick {
  public:
  	Joystick(uint8_t, uint8_t);
  
  	short getX();
  	short getY();
  
  	E_JOYSTICK getStateX();
  	E_JOYSTICK getStateY();
  private:
  	uint8_t input_x;
  	uint8_t input_y;
};

Joystick::Joystick(uint8_t input_x, uint8_t input_y) {
  pinMode(input_x, INPUT);
  pinMode(input_y, INPUT);
  
  this->input_x = input_x;
  this->input_y = input_y;
}

short Joystick::getX() {
  return analogRead(this->input_x);
}

short Joystick::getY() {
  return analogRead(this->input_y);
}

E_JOYSTICK Joystick::getStateX() {
  if(this->getX() > LIMIT_RIGHT) {
  	return E_JOYSTICK::RIGHT;
  } else if(this->getX() < LIMIT_LEFT) {
  	return E_JOYSTICK::LEFT;
  }
  return E_JOYSTICK::NONE;
}

E_JOYSTICK Joystick::getStateY() {
  if(this->getY() > LIMIT_UP) {
  	return E_JOYSTICK::UP;
  } else if(this->getY() < LIMIT_DOWN) {
  	return E_JOYSTICK::DOWN;
  }
  return E_JOYSTICK::NONE;
}

/////////////////////////////////

class Motor {
  public:
  	Motor(Joystick *, uint8_t, short, short);
  	void write(short angle);
  	short velocity_map(short, short, short, short, short);
  	virtual void update();
  protected:
  	Servo s_motor;
  	Joystick *joystick;
  	uint8_t output;
  	short bottom_angle, top_angle;
  	short current_angle;
};

Motor::Motor(Joystick *joystick,
             uint8_t output, 
             short bottom_angle, 
             short top_angle) {
  this->joystick = joystick;
  this->output = output;
  this->bottom_angle = bottom_angle;
  this->top_angle = top_angle;
  
  this->s_motor.attach(this->output);
  
  // initializa todos os motores a 90º
  this->current_angle = 90;
  this->write(this->current_angle);
};

void Motor::update() {
  // é uma função virtual, será sobrescrito
}

void Motor::write(short angle) {
  Serial.print("ANGLE: ");
  Serial.println(angle);
  
  this->s_motor.write(current_angle);
}

short Motor::velocity_map(short value, short left, short right, short v_i, short v_f) {
	return map(value, left, right, v_i, v_f);
}

/////////////////////////////////

class MotorBase : public Motor {
  public:
  	MotorBase(Joystick *joystick,
              uint8_t output, 
              short bottom_angle, 
              short top_angle) 
      : Motor(joystick, output, bottom_angle, top_angle) {};
  	void update();
};

class MotorDireito : public Motor {
  public:
  	MotorDireito(Joystick *joystick,
              uint8_t output, 
              short bottom_angle, 
              short top_angle) 
      : Motor(joystick, output, bottom_angle, top_angle) {}
  	void update();
};

class MotorEsquerdo : public Motor {
  public:
  	MotorEsquerdo(Joystick *joystick, 
              uint8_t output,
              short bottom_angle, 
              short top_angle) 
      : Motor(joystick, output, bottom_angle, top_angle) {}
  	void update();
};

class MotorGarra : public Motor {
  public:
  	MotorGarra(Joystick *joystick,
              uint8_t output, 
              short bottom_angle, 
              short top_angle) 
      : Motor(joystick, output, bottom_angle, top_angle) {}
  	void update();
};

void MotorBase::update() {
  E_JOYSTICK state = this->joystick->getStateX();
  short x = this->joystick->getX();
  short velocity = 0;
  
  bool has_limit = false;
  
  if(state == E_JOYSTICK::RIGHT && this->current_angle < this->top_angle) {
   	velocity = this->velocity_map(x, LIMIT_UP, 1000, 0, 4);
    if(this->current_angle + velocity > this->top_angle) {
      velocity = 0;
      this->current_angle = this->top_angle;
      has_limit = true;
    }
  } else if(state == E_JOYSTICK::LEFT && this->current_angle > this->bottom_angle) {
  	velocity = this->velocity_map(x, 0, LIMIT_DOWN, -4, 0);
    if(this->current_angle + velocity < this->bottom_angle) {
      velocity = 0;
      this->current_angle = this->bottom_angle;
      has_limit = true;
    }
  }
  
  if(velocity != 0 || has_limit) {
  	Serial.print("VELOCITY_ANGLE (X - MotorBase): ");
  	Serial.println(velocity);
  
  	this->current_angle += velocity;
  	this->write(this->current_angle);
  }
}

void MotorDireito::update() {
  E_JOYSTICK state = this->joystick->getStateX();
  short x = this->joystick->getX();
  short velocity = 0;

  bool has_limit = false;
  
 if(state == E_JOYSTICK::RIGHT && this->current_angle < this->top_angle) {
   	velocity = this->velocity_map(x, LIMIT_UP, 1000, 0, 4);
    if(this->current_angle + velocity > this->top_angle) {
      velocity = 0;
      this->current_angle = this->top_angle;
      has_limit = true;
    }
  } else if(state == E_JOYSTICK::LEFT && this->current_angle > this->bottom_angle) {
  	velocity = this->velocity_map(x, 0, LIMIT_DOWN, -4, 0);
    if(this->current_angle + velocity < this->bottom_angle) {
      velocity = 0;
      this->current_angle = this->bottom_angle;
      has_limit = true;
    }
  }
  
  if(velocity != 0 || has_limit) {
    Serial.print("VELOCITY_ANGLE (X - MotorDireito): ");
    Serial.println(velocity);

    this->current_angle += velocity;
    this->write(this->current_angle);
  }
}

void MotorEsquerdo::update() {
  E_JOYSTICK state = this->joystick->getStateY();
  short y = this->joystick->getY();
  short velocity = 0;
  
  bool has_limit = false;
  
  if(state == E_JOYSTICK::UP && this->current_angle < this->top_angle) {
   	velocity = this->velocity_map(y, LIMIT_UP, 1000, 0, 4);
    if(this->current_angle + velocity > this->top_angle) {
      velocity = 0;
      this->current_angle = this->top_angle;
      has_limit = true;
    }
  } else if(state == E_JOYSTICK::DOWN && this->current_angle > this->bottom_angle) {
  	velocity = this->velocity_map(y, 0, LIMIT_DOWN, -4, 0);
    if(this->current_angle + velocity < this->bottom_angle) {
      velocity = 0;
      this->current_angle = this->bottom_angle;
      has_limit = true;
    }
  }
  
  if(velocity != 0 || has_limit) {
  	Serial.print("VELOCITY_ANGLE (Y - MotorEsquerdo): ");
  	Serial.println(velocity);
  
  	this->current_angle += velocity;
  	this->write(this->current_angle);
  }
}

void MotorGarra::update() {
  E_JOYSTICK state = this->joystick->getStateY();
  short y = this->joystick->getY();
  short velocity = 0;
  
  bool has_limit = false;
  
  if(state == E_JOYSTICK::UP && this->current_angle < this->top_angle) {
   	velocity = this->velocity_map(y, LIMIT_UP, 1000, 0, 4);
    if(this->current_angle + velocity > this->top_angle) {
      velocity = 0;
      this->current_angle = this->top_angle;
      has_limit = true;
    }
  } else if(state == E_JOYSTICK::DOWN && this->current_angle > this->bottom_angle) {
  	velocity = this->velocity_map(y, 0, LIMIT_DOWN, -4, 0);
    if(this->current_angle + velocity < this->bottom_angle) {
      velocity = 0;
      this->current_angle = this->bottom_angle;
      has_limit = true;
    }
  }
  
  if(velocity != 0 || has_limit) {
  	Serial.print("VELOCITY_ANGLE (Y - MotorGarra): ");
  	Serial.println(velocity);
  
  	this->current_angle += velocity;
  	this->write(this->current_angle);
  }
}

#define QUANTIDADE_MOTORES 4
Motor *motores[QUANTIDADE_MOTORES];

void setup() {
  Serial.begin(9600);
  
  // cria o joystick 1 e o joystick 2
  Joystick *joystick_1 = new Joystick(JOYSTICK_X1, JOYSTICK_Y1);
  Joystick *joystick_2 = new Joystick(JOYSTICK_X2, JOYSTICK_Y2);
  
  // associa cada motor com seu pino e seu joystick
  motores[0] = new MotorBase(joystick_1, MOTOR_BASE, 0, 180);
  motores[1] = new MotorDireito(joystick_2, MOTOR_DIREITO, 45, 135);
  motores[2] = new MotorEsquerdo(joystick_2, MOTOR_ESQUERDO, 50, 90);
  motores[3] = new MotorGarra(joystick_1, MOTOR_GARRA, 90, 130);
}

void loop() {
  // atualiza todos os motores
  for(short i = 0; i < QUANTIDADE_MOTORES; ++i) {
    motores[i]->update();
  }
  delay(100);
}