// 19/04/2015(Dom)
#include <avr/pgmspace.h>

static const char ABOUT[][6] PROGMEM={"SOBRE", "ABOUT"};
static const char FW_VERSION[] PROGMEM="FW: 1.8a 20150701";
static const char HW_VERSION[] PROGMEM="HW: 1.2a 20180626";
static const char MSG_LINUX[] PROGMEM="F\5bio (Linux) \7";
static const char STR_MENU[][9] PROGMEM={"1-Time","2-Pins","5-Others","6-About","1-Hora","2-Pinos","5-Outros","6-Sobre", "3-Sensor","4-Calc."};
static const char STR_MENU_TITLE[] PROGMEM=" MENU ";
static const char WEEKDAY_EN[][4] PROGMEM={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
static const char WEEKDAY_BR[][4] PROGMEM={"Dom","Seg","Ter","Qua","Qui","Sex","S\5b"};
static const char MONTH_EN[][4] PROGMEM={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
static const char MONTH_BR[][4] PROGMEM={"Jan","Fev","Mar","Abr","Mai","Jun","Jul","Ago","Set","Out","Nov","Dez"};
static const char DATE_TIME_ADJUST_EN[] PROGMEM="DATE/TIME ADJUST";
static const char DATE_TIME_EN[] PROGMEM="1-Date/time";
static const char DATE_TIME_WEEKDAY_EN[] PROGMEM="2-Weekday";
static const char DATE_TIME_ALARM_EN[] PROGMEM="3-Alarm";
static const char DATE_TIME_ADJUST_BR[] PROGMEM="AJUST. DATA E HORA";
static const char DATE_TIME_BR[] PROGMEM="1-Data/hora";
static const char DATE_TIME_WEEKDAY_BR[] PROGMEM="2-Dia da semana";
static const char DATE_TIME_ALARM_BR[] PROGMEM="3-Alarme";
//static const char TYPE_STR_EN[] PROGMEM="Type";
//static const char TYPE_STR_BR[] PROGMEM="Digite";
static const char TYPE_STR[][7] PROGMEM={"Digite", "Type"};
static const char CENTURY_19XX[] PROGMEM="19";
static const char CENTURY_20XX[] PROGMEM="20";
static const char WEEKDAY_MENU_STR_EN[] PROGMEM="WEEKDAY";
static const char WEEKDAY_MENU_STR_BR[] PROGMEM="DIA DA SEMANA";
//static const char WEEKDAY_MENU_STR_YYYYMMDD[] PROGMEM=": yyyymmdd";

static const char WRITING_MSG[][9] PROGMEM={"Gravando", "Writing"};
static const char STR_SENSOR[][8] PROGMEM={"s/ bat.", "no bat."};
static const char ALARM_STR[][7] PROGMEM={"ALARME","ALARM"};
//static const char ALARM_TYPE_STR[] PROGMEM=":chhmm:";

static const char ONE_HUNDRED_STR[] PROGMEM="100";

static const char STATUS_OK[] PROGMEM="OK!";
static const char ERR_INVALID[] PROGMEM="Invalid ";
static const char ERR_TIME[][5] PROGMEM={"Hora","Time"};
static const char ERR_DATE[] PROGMEM="date";
static const char ERR_YEAR[] PROGMEM="year";

//static const char ERR_TIME_BR[] PROGMEM="Hora";

static const char PINS_CONF_MENU_STR_EN[] PROGMEM="PINS CONF.";
static const char PINS_CONF_MENU_STR_BR[] PROGMEM="CONF. DE PINOS";
static const char PINS_CONF_MENU_STR_A[] PROGMEM="1-P1   3-P3   5-P5";
static const char PINS_CONF_MENU_STR_B[] PROGMEM="2-P2   4-P4   6-P6";
static const char PIN_P_STR[][7] PROGMEM={"Pino P","Pin P"};

static const char ERR_OVRFLW[] PROGMEM="Err:ovrflw";
static const char ERR_TIME2[] PROGMEM="Err:time";
static const char ERR_MINUTE[] PROGMEM="Err:minute";
static const char ERR_SEC[] PROGMEM="Err:sec.";

static const char ACCESS_DENIED[][14] PROGMEM={"Acesso negado","Access denied"};

static const char ERR_ON_OFF[] PROGMEM="Err:ON-OFF";

static const char ERR_E_D[] PROGMEM="Err:E/D";
static const char ERR_X2_Y2[] PROGMEM="Err:x2/y2";

static const char ERR_C_F[] PROGMEM="Err:C/F";
static const char ERR_BAT_1_2[] PROGMEM="Err.:BAT1/2";
static const char ERR_CODE[] PROGMEM="Err:code";

static const char ERR_ALARM_CONF_STR[] PROGMEM="Err:alarm conf.";
static const char ERR_INPUT_STR[] PROGMEM="Err:input";

static const char CALCULA_STR[] PROGMEM="CALCULA";
/*
static const char CALCULA_STR2[] PROGMEM="TOR";
static const char CALCULA_STR3[] PROGMEM="DORA";
static const char DEF_CALC_EN[] PROGMEM="Default calc.";
static const char SCI_CALC_EN[] PROGMEM="Scientific calc.";

static const char DEF_CALC_BR[] PROGMEM="Calc. padr\1o";
static const char SCI_CALC_BR[] PROGMEM="Calc. cient\3fica";
*/

static const char CALCULA_STR2[][5] PROGMEM={"DORA","TOR"};
static const char DEF_CALC[][14] PROGMEM={"Calc. padr\1o", "Default calc."};
static const char SCI_CALC[][17] PROGMEM={"Calc. cient\3fica","Scientific calc."};

static const char OTHER_CONF_STR[][13] PROGMEM={"OUTRAS CONF.","OTHER CONF."};
static const char LANGUAGE_STR[] PROGMEM="1-Language/Idioma";
static const char TEMP_MEASURE[][16] PROGMEM={"Sensor de temp.","Temp. sensor"};
static const char UART_FOR_IOT_MENU_STR[] PROGMEM="UART for IoT";
static const char UART_UART2X_STR[] PROGMEM="UART2x";
static const char UART_UART2X_DISABLE_STR[][10] PROGMEM={"Desligado", "Disabled"};
static const char UART_UART2X_ENABLED_STR[][8] PROGMEM={"Ligado", "Enabled"};
static const char UART_UART2X_STATUS_STR[] PROGMEM="Status: ";
static const char UART_SPEED_CONFIGURATION_STR[][7] PROGMEM={"Veloc.", "Speed"};
static const char UART_PARITY_CONFIGURATION_STR[][9] PROGMEM={"Paridade","Parity"};
static const char UART_DEFAULT_CONFIGURATION_STR[][8] PROGMEM={"Padr\1o","Default"};
static const char UART_DEFAULT_CONF_QUESTION_STR[][10] PROGMEM={"Carregar ", "Load "};
static const char UART_ROOT_ACCESS_PERMISSION_STR[][11] PROGMEM={"Permiss\1o","Permission" };
//static const char QUESTION_STR PROGMEM='?';
static const char YES_STR[][4] PROGMEM={"Sim","Yes"};
static const char NO_STR[][4] PROGMEM={"N\1o","No"};
static const char SET_PASSWORD[][14] PROGMEM={"Definir SENHA","Set PASSWORD"};
static const char OLD_PASSWORD_STR[][13] PROGMEM={"Senha atual","Old password"};
static const char NEW_PASSWORD_STR[][13] PROGMEM={"Nova senha", "New password"};
static const char RETYPE_PASSWORD_STR[][16] PROGMEM={"Repetir senha", "Retype password"};
static const char PASSWD_MISMATCH_STR[][18] PROGMEM={"Senha n\1o confere", "Type mismatch"};
static const char PASSWORD_STR[][9] PROGMEM={"Senha", "Password"};
static const char SET_SECURITY[][18] PROGMEM={"Atributos de seg.", "Security attrib."};
static const char INDEX_MARK[] PROGMEM="[ ]";
static const char UART_PARITY_NONE[][7] PROGMEM={"Nenhum","None"};
static const char UART_PARITY_ODD[][4] PROGMEM={"Par","Odd"};
static const char UART_PARITY_EVEN[][6] PROGMEM={"\4mpar", "Even"};
static const char SET_CONFIGURE_PORT[][17] PROGMEM={"Configurar porta", "Configure port"};
static const char LANG[][12] PROGMEM={"2-<BR-PORT>", "1-<US-ENGL>"};
static const char LANG_STARTUP[][10] PROGMEM={"Iniciando","Starting"};

static const char MATH_EXP_STR[] PROGMEM="exp";
static const char MATH_LN_STR[] PROGMEM="ln";
static const char MATH_LN_X_C_STR[] PROGMEM="ln(x)/C";
static const char MATH_SQRT_STR[] PROGMEM="sqrt";
static const char MATH_CBRT_STR[] PROGMEM="cbrt";
static const char MATH_SIN_STR[] PROGMEM="sin";
static const char MATH_COS_STR[] PROGMEM="cos";
static const char MATH_TAN_STR[] PROGMEM="tan";
static const char MATH_SIND_STR[] PROGMEM="sind";
static const char MATH_COSD_STR[] PROGMEM="cosd";
static const char MATH_TAND_STR[] PROGMEM="tand";
static const char MATH_ASIN_STR[] PROGMEM="asin";
static const char MATH_ACOS_STR[] PROGMEM="acos";
static const char MATH_ATAN_STR[] PROGMEM="atan";
static const char MATH_ASIND_STR[] PROGMEM="asind";
static const char MATH_ACOSD_STR[] PROGMEM="acosd";
static const char MATH_ATAND_STR[] PROGMEM="atand";
static const char MATH_C_A_STR[] PROGMEM="C<-A";
static const char MATH_A_C_STR[] PROGMEM="A<-C";
static const char MATH_HYP_STR[] PROGMEM="hyp(C,A)";
static const char MATH_MOD_STR[] PROGMEM="mod(C,A)";
static const char MATH_SINH_STR[] PROGMEM="sinh";
static const char MATH_COSH_STR[] PROGMEM="cosh";
static const char MATH_TANH_STR[] PROGMEM="tanh";
static const char MATH_F_X_STR[] PROGMEM="f(x)=";
static const char MATH_X_EQUAL[] PROGMEM="(x) =";
static const char MATH_A_STR[] PROGMEM="A ";
static const char MATH_B_EQUALS_C_STR[] PROGMEM=" B = C";

// UART messages max 64 chars
static const char ERR_UART_DISABLE[] PROGMEM="UART disabled. Contact root user for privileges";
static const char ERR_MEMORY_OVF[] PROGMEM="Memory out of range[0-"; // 22 chars
static const char ERR_MEMORY_OVF_EEPROM[] PROGMEM="1016]";
static const char ERR_MEMORY_OVF_SRAM[] PROGMEM="2047]";
static const char ERR_MEMORY_OVF_FLASH[] PROGMEM="32255]";
static const char ERR_INVALID_UART_COMMAND[] PROGMEM="Invalid UART command";
static const char ERR_TIME_OUT[] PROGMEM="Time out. I can't wait forever :(";
static const char ERR_I2C_BUSY[] PROGMEM="I2C busy. Send [STOP] command to unlock";
//static const char ERR_LCD_PANEL PROGMEM="for LCD panel";
static const char LCD_MODE_MSG[] PROGMEM="LCD mode ready";
static const char LCD_MODE_OFF_MSG[] PROGMEM="Exiting UART/LCD";

static const char AUTHOR_NAME[] PROGMEM="Fábio Pereira da Silva";// YES ! in utf8 format
static const char AUTHOR_EMAIL[] PROGMEM="fabioegel@gmail.com";
static const char USER[] PROGMEM="fabioUSER";

static const char ERR_BCD_FORMAT[] PROGMEM="Error in BCD format";
static const char ERR_YEAR_INTERVAL[] PROGMEM="- Year must be 1900-2099.";

static const char ERR_UART_SECURUTY_LOCK[] PROGMEM="UART for IoT is temporary locked by user";
static const char ERR_UART_FORBIDDEN_ACCESS[] PROGMEM="FORBIDDEN: Only ROOT user has permission to access it.";

//static const unsigned int UBRR[]={};

