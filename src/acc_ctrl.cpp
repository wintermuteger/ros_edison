#include "acc_ctrl.h"

int i2c_file = 0;

//#define IIR_DIV 3 
//#define IIR_RES 2

//int16_t res_fil_x = 0;
//int16_t res_fil_y = 0; //IIR filtering of the Y-Acc output
//int16_t res_fil_z = 0;

int acc_init()
{ 
  printf("Setting I2C mux...\n");
  set_gpio(214,0);
  set_gpio(204,0);
  set_gpio(205,0);
  
  sys_write("/sys/class/gpio/export", 14);
  sys_write("/sys/class/gpio/gpio%d/direction",14,"in");
  sys_write("/sys/class/gpio/unexport", 14);
  sys_write("/sys/class/gpio/export", 165);
  sys_write("/sys/class/gpio/gpio%d/direction",165,"in");
  sys_write("/sys/class/gpio/unexport", 165);

  set_gpio(236,0);
  set_gpio(237,0);

  sys_write("/sys/class/gpio/export", 212);
  sys_write("/sys/class/gpio/gpio%d/direction",212,"in");
  sys_write("/sys/class/gpio/unexport", 212);
  sys_write("/sys/class/gpio/export", 213);
  sys_write("/sys/class/gpio/gpio%d/direction",213,"in");
  sys_write("/sys/class/gpio/unexport", 213);

  set_SoC_mode(28, "mode1");
  set_SoC_mode(27, "mode1");
  set_gpio(214,1);

  //printf("Load Kernel driver module...\n");
  //system("sudo modprobe i2c-dev");
  //system("sudo chmod o+rw /dev/i2c*");
  printf("Read test register for 0x68...\n");
  //system("i2cget -y 0 0x68 0x75 b");
  const unsigned int FILENAME_SIZE = 64;
  char filename[FILENAME_SIZE];
  int file = open_i2c_dev(I2C_ID, filename, FILENAME_SIZE, 0);
  if(file <= 0)
    {
      printf("ERROR: I2C file system representation not accesible!\n");
      return 0;
    }
  set_slave_addr(file, 0x68, 0);
  int res = i2c_smbus_read_byte_data(file,0x75);
  printf("Result in hexa is  0x%X\n", res);
  
  //Switch accelerometer to internal oscillator
  printf("Switch accelerometer to internal oscillator...\n");
  i2c_smbus_write_byte_data(file,0x6B,0x00);

  //Store verified file as global variable
  i2c_file = file;

  return 1;
}

int acc_shutdown()
{
  close_i2c_dev();
  return 1;
}

int open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet)
{
	int file;

	snprintf(filename, size, "/dev/i2c/%d", i2cbus);
	filename[size - 1] = '\0';
	file = open(filename, O_RDWR);

	if (file < 0 && (errno == ENOENT || errno == ENOTDIR)) {
		sprintf(filename, "/dev/i2c-%d", i2cbus);
		file = open(filename, O_RDWR);
	}

	if (file < 0 && !quiet) {
		if (errno == ENOENT) {
			fprintf(stderr, "Error: Could not open file "
				"`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
				i2cbus, i2cbus, "ENOENT");
		} else {
			fprintf(stderr, "Error: Could not open file "
				"`%s': %s\n", filename, "errno");
			if (errno == EACCES)
				fprintf(stderr, "Run as root?\n");
		}
	}

	return file;
}

int set_slave_addr(int file, int address, int force)
{
	/* With force, let the user read from/write to the registers
	   even when a driver is also running */
	if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
		fprintf(stderr,
			"Error: Could not set address to 0x%02x: %s\n",
			address, "errno");
		return -errno;
	}

	return 0;
}

int close_i2c_dev()
{

  printf("Closing i2c file handler...\n");
  if(i2c_file == 0)
    {
      close(i2c_file);
      i2c_file = 0;
    }

  return 1;
}


int16_t read_Y_acc()
{
  if(i2c_file == 0)
    return 0;
  
  int16_t res_h = i2c_smbus_read_byte_data(i2c_file,0x3D);
  int16_t res_l = i2c_smbus_read_byte_data(i2c_file,0x3E);

  int16_t res = ((res_h<<8)|res_l);

  //Perform simple IIR filtering
  //res_fil_y = IIR_RES*res_fil_y/IIR_DIV + res/IIR_DIV;

  return res;
}

int16_t read_Z_acc()
{
  if(i2c_file == 0)
    return 0;
  
  int16_t res_h = i2c_smbus_read_byte_data(i2c_file,0x3F);
  int16_t res_l = i2c_smbus_read_byte_data(i2c_file,0x40);

  int16_t res = ((res_h<<8)|res_l);

  //Perform simple IIR filtering
  //res_fil_z = IIR_RES*res_fil_z/IIR_DIV + res/IIR_DIV;

  return res;
}


int16_t read_X_acc()
{
  if(i2c_file == 0)
    return 0;
  
  int16_t res_h = i2c_smbus_read_byte_data(i2c_file,0x3B);
  int16_t res_l = i2c_smbus_read_byte_data(i2c_file,0x3C);

  int16_t res = ((res_h<<8)|res_l);

  //Perform simple IIR filtering
  //res_fil_x = IIR_RES*res_fil_x/IIR_DIV + res/IIR_DIV;

  return res;
}

int16_t read_X_gyro()
{
  if(i2c_file == 0)
    return 0;
  
  int16_t res_h = i2c_smbus_read_byte_data(i2c_file,0x43);
  int16_t res_l = i2c_smbus_read_byte_data(i2c_file,0x44);

  int16_t res = ((res_h<<8)|res_l);

  //Perform simple IIR filtering
  //res_fil_x = IIR_RES*res_fil_x/IIR_DIV + res/IIR_DIV;

  return res;
}

int16_t read_Y_gyro()
{
  if(i2c_file == 0)
    return 0;
  
  int16_t res_h = i2c_smbus_read_byte_data(i2c_file,0x45);
  int16_t res_l = i2c_smbus_read_byte_data(i2c_file,0x46);

  int16_t res = ((res_h<<8)|res_l);

  //Perform simple IIR filtering
  //res_fil_x = IIR_RES*res_fil_x/IIR_DIV + res/IIR_DIV;

  return res;
}

int16_t read_Z_gyro()
{
  if(i2c_file == 0)
    return 0;
  
  int16_t res_h = i2c_smbus_read_byte_data(i2c_file,0x47);
  int16_t res_l = i2c_smbus_read_byte_data(i2c_file,0x48);

  int16_t res = ((res_h<<8)|res_l);

  //Perform simple IIR filtering
  //res_fil_x = IIR_RES*res_fil_x/IIR_DIV + res/IIR_DIV;

  return res;
}
