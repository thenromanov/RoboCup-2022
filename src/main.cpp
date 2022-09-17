#include <Arduino.h>
#include <Pixy2.h>
#include <Robot.h>
#include <config.h>

const int sensor_digitals[] = {28, 29, 30};
const int sensor_analogs[] = {A12, A13};
const int sensor_order[] = {10, 8, 4,  5,  9, 11, 7,  6,
                            0,  1, 12, 13, 2, 3,  15, 14};
const int sensor_grey[] = {0,   600, 603, 608, 602, 601, 604, 609,
                           600, 600, 608, 600, 608, 604, 602, 608};

const double pk = 240;

Button b[3] = {Button(39), Button(14), Button(15)};

Leds led(36, 4);

Motor m[4] = {Motor(3, 4, 5), Motor(2, 7, 6), Motor(8, 26, 25),
              Motor(9, 24, 10)};

Gyro gyro(17, 16);

Kicker kicker(22);

Interrupter inter(A16);

Pixy2 pixy;

Cam_Block ball, home, goal, center(166, 127);

const Vect basis(1, 0);

Sensor sens(sensor_digitals, sensor_analogs, sensor_order, sensor_grey);

struct Robot {
  double speed, angle, heading;
  uint64_t line_tmr;
  bool on_line;
};

Robot bot;

void move(int speed, double angle, double k) {
  m[0].start(speed * cos(angle + 0.78539816339744830961566) + k);
  m[1].start(speed * cos(angle - 0.78539816339744830961566) + k);
  m[2].start(speed * -cos(angle + 0.78539816339744830961566) + k);
  m[3].start(speed * -cos(angle - 0.78539816339744830961566) + k);
}

bool check_timer(uint64_t t, uint64_t dt) { return (millis() - t < dt); }

double adduced_dir(double dir) {
  if (!ball.found)
    return PI;
  else if (abs(dir) < PI / 14)
    return 0;
  return dir + 45 / ball.dist * (1 - 2 * (dir < 0));
}

void update_cam() {
  ball.found = false;
  goal.found = false;
  home.found = false;
  pixy.ccc.getBlocks(false, CCC_SIG1 | CCC_SIG2 | CCC_SIG3);
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 1 && !ball.found) {
      ball.x = pixy.ccc.blocks[i].m_x;
      ball.y = pixy.ccc.blocks[i].m_y;
      ball.w = pixy.ccc.blocks[i].m_width;
      ball.h = pixy.ccc.blocks[i].m_height;
      ball.angle = Vect(center, ball) ^ basis;
      ball.dist = Vect(center, ball).get_length();
      ball.found = true;
      ball.time = millis();
    }
    if (pixy.ccc.blocks[i].m_signature == GOAL && !goal.found) {
      goal.x = pixy.ccc.blocks[i].m_x;
      goal.y = pixy.ccc.blocks[i].m_y;
      goal.w = pixy.ccc.blocks[i].m_width;
      goal.h = pixy.ccc.blocks[i].m_height;
      goal.angle = Vect(center, goal) ^ basis;
      goal.dist = Vect(center, goal).get_length();
      goal.found = true;
      goal.time = millis();
    }
    if (pixy.ccc.blocks[i].m_signature == HOME && !home.found) {
      home.x = pixy.ccc.blocks[i].m_x;
      home.y = pixy.ccc.blocks[i].m_y;
      home.w = pixy.ccc.blocks[i].m_width;
      home.h = pixy.ccc.blocks[i].m_height;
      home.angle = Vect(center, home) ^ basis;
      home.dist = Vect(center, home).get_length();
      home.found = true;
      home.time = millis();
    }
  }
  ball.found = check_timer(ball.time, 200);
  home.found = check_timer(home.time, 200);
  goal.found = check_timer(goal.time, 200);
}

int sensor_range(int n, int l, int r) {
  return (n >> l) & ((1 << (r - l + 1)) - 1);
}

void setup() {
  SPI.setSCK(27);
  Serial.begin(115200);
  pixy.init();
  for (int i = 0; i < 4; i++) led.turn_off(i);
}

void loop() {
  kicker.kick(false);
  update_cam();
  int state = sens.read();
  if (state) bot.line_tmr = millis();
  bot.on_line = check_timer(bot.line_tmr, 50);
  bot.speed = 280;
  bot.angle = adduced_dir(ball.angle);
  double gamma = gyro.read();
  bot.heading = gamma * 180;
  if (goal.found) bot.heading = -goal.angle * 270;
  if (bot.on_line || abs(gamma) > PI / 5.2)
    bot.angle =
        ((goal.found && goal.dist > home.dist) || !home.found ? goal.angle
                                                              : home.angle);
  if (!ball.found) {
    if (home.dist < 90) {
      bot.speed = 250;
      bot.angle = 0;
    } else if (home.dist > 100) {
      bot.speed = 250;
      bot.angle = home.angle;
    } else {
      bot.speed = 250 * abs(sin(home.angle));
      bot.angle = PI / 2 * (1 - 2 * (home.angle < 0));
    }
  } else if (goal.dist < 68) {
    bot.angle = home.angle;
  }
  move(bot.speed, bot.angle, bot.heading);
  if (inter.read() < 50) kicker.kick(true);
}