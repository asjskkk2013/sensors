#ifndef HMC5883L_H
#define HMC5883L_H
 typedef struct {
  int raw_x;
  int raw_y;
  int raw_z;
  float x;
  float y;
  float z;
} HMC5883L_Data;

void hmc5883l_init(void);
HMC5883L_Data HMC5883L_ReadData();
void ShowHMC5883L(void);

#endif