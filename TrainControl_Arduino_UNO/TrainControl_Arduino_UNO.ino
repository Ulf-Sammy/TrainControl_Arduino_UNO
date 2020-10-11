/*
 Name:		TrainControl_Arduino_UNO.ino
 Created:	04.10.2020 13:37:05
 Author:	Ulf
 Melden wenn Züge ist im Block
 Blöcke mit Spannung versorgen

*/
#include <I2C.h>
#include <LiquidCrystal.h>

#define MAX_COMMAND_LENGTH 100    

#define KEYPAD_KEY_RIGHT  0
#define KEYPAD_KEY_UP     1
#define KEYPAD_KEY_DOWN   2
#define KEYPAD_KEY_LEFT   3
#define KEYPAD_KEY_SELECT 4
#define KEYPAD_KEY_NONE   5
enum theKeys {Right,Up,Down,Left,Select,None};


#define KEYPAD_KEY_RIGHT_ADC_LOW   0
#define KEYPAD_KEY_RIGHT_ADC_HIGH  20
#define KEYPAD_KEY_UP_ADC_LOW      120
#define KEYPAD_KEY_UP_ADC_HIGH     140
#define KEYPAD_KEY_DOWN_ADC_LOW    290
#define KEYPAD_KEY_DOWN_ADC_HIGH   339
#define KEYPAD_KEY_LEFT_ADC_LOW    450
#define KEYPAD_KEY_LEFT_ADC_HIGH   520
#define KEYPAD_KEY_SELECT_ADC_LOW  700
#define KEYPAD_KEY_SELECT_ADC_HIGH 780
#define KEYPAD_KEY_NONE_ADC_LOW    1000
#define KEYPAD_KEY_NONE_ADC_HIGH   102
#define PIN_KEY A0
#define PIN_BACKLIGHT 10

#define COM_LEN(L)      (L & 0x07)     //                       Setup - Test - Auto - Program
#define COM_WRITE_MOD       0x08+0x03  // 1 MOD	+ Sub				| S | A | P | modus des Prog
#define COM_SEND_BLOCK      0x90+0x03  //18 Block + Wert			| _ | T | A | _ |
#define COM_SEND_RELAIS     0x78+0x02  //15 Relais + Bit			| _ | T | A | _ |
#define COM_WRITE_RELAIS    0x80+0x02  //16 Relais + Bit			| _ | T | A | _ |;


const byte  MaxMelderGleis = 5;
const byte  MaxRelaisGleis = 1;
const byte  Relais  =  0x3D;

struct Relais_info
{
    bool ON = false;
    byte Data = 0xFF;
    byte Adr = 0x00;
};
struct Melder_Info
{
    bool ON = false;
    byte Data = 0xFF;
    byte Adr = 0x00;
    //unsigned long TimeUpdate;
};


LiquidCrystal LCD(8, 9, 4, 5, 6, 7);
volatile Melder_Info MelderGruppe[MaxMelderGleis];
volatile Melder_Info RelaisGruppe[MaxRelaisGleis];
volatile bool  BlockMelder[5 * 8];

int ErrorZaehler = 0;
byte Send_Data[3];
byte Get_Data[3];
byte gefundeneMelder = 0;
byte gefundeneRelais = 0;
byte Befehl;
byte Station = 0;
theKeys last_key, key;
byte Block_Test_Nr = 0;
byte Block_Test_Nr_alt = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("  I2C Melder V2.0");
    LCD.begin(16, 2);
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("I2C Melder V2.0");
    // Initialize I2C library manually
    I2c.begin();
    I2c.timeOut(500);
    I2c.pullup(true);
    do
    {
        if (Serial.available() > 0)
        {
            Befehl = Serial.read();
        }
    } while (Befehl != COM_WRITE_MOD);
}


void loop() 
{
    key =  (theKeys) read_key();
    if (key != last_key)
    {
        last_key = key;

        switch (key)
        {
        case Right:
            LCD.setCursor(0, 1);
            LCD.print("right key");
            break;
        case Up:
            Block_Test_Nr_alt = Block_Test_Nr;
            Block_Test_Nr++;
            LCD.setCursor(0, 1);
            LCD.print("up Key");
            break;
        case Down:
            Block_Test_Nr_alt = Block_Test_Nr;
            Block_Test_Nr--;
            LCD.setCursor(0, 1);
            LCD.print("down Key");
            break;
        case Left:
            LCD.setCursor(0, 1);
            LCD.print("left Key");
            break;
        case Select:
            LCD.setCursor(0, 1);
            LCD.print("select Key");
            break;
        case None:
        default:
            LCD.setCursor(0, 0);
            LCD.print("Block : ");
            LCD.print(Block_Test_Nr, 10);
            LCD.print("        ");
            Test_Melder();
            LCD.setCursor(0, 1);
            LCD.print("                      ");
            break;
        }
    }
    /*
    Befehl = Hole_Neuen_Befehl();
    if (Befehl == COM_WRITE_RELAIS)
    {
        SchalteRelais(Get_Data[1], (bool)Get_Data[2]);
    }
    if (MelderGruppe[Station].ON)
    {
        ReadMelder(Station);
        Melde_Melder(Station);
    }
    Station++;
    if (Station == gefundeneMelder)
    {
        Station == 0;
    }
    */
}


void setup_Melder()
{
    byte Data;

    MelderGruppe[0].Adr = 0x20; // Strecke im Garten
    MelderGruppe[1].Adr = 0x21; // Strecke im Garten
    MelderGruppe[2].Adr = 0x22; // Strecke im Garten
    MelderGruppe[3].Adr = 0x23; // leer
    MelderGruppe[4].Adr = 0x24; // Im Lokschuppen
    for (byte i = 0; i < MaxMelderGleis; i++)
    {
        if (I2c.read(MelderGruppe[i].Adr, 0x01, &Data) == 0)
        {
            gefundeneMelder++;
            Data = (0xFF ^ Data);
            MelderGruppe[i].Data = Data;
            MelderGruppe[i].ON = true;
        }
    }
    Serial.println(F("Fertig mit Scan !"));
}

byte Setup_Relais()
{
    byte Data;
    
    for (byte i = 0; i < MaxRelaisGleis; i++)
    {
        if (I2c.read(MelderGruppe[i].Adr, 0x01, &Data) == 0)
        {
            gefundeneRelais++;
            Data = (0xFF ^ Data);
            MelderGruppe[i].Data = Data;
            MelderGruppe[i].ON = true;
        }
    }
    return gefundeneRelais;
}
void SchalteRelais(byte Nr, bool Bit)
{
    static byte R_Data = 0xFF;
    if (Bit)    bitClear(R_Data, Nr);
    else        bitSet(R_Data, Nr);
    I2c.write(Relais, 0x00, R_Data);
    Send_Data[0] = COM_SEND_RELAIS;
    Send_Data[1] = Nr;
    Send_Data[2] = (byte)Bit;
    Serial.write(Send_Data, COM_LEN(Send_Data[0]));
    delay(100);
}
byte Hole_Neuen_Befehl()
{
    static int Data;
    static byte n = 0;
    static byte Len;

    if (Serial.available() > 0)
    {
        for (;;)
        {
            Data = Serial.read();
            Get_Data[n] = Data;
            if (n == 0) { Len = COM_LEN(Get_Data[n]); }
            if (n == Len)
            {
                n = 0;
                break;
            }
            if (Data != -1) n++;
        }
        return (Get_Data[0]);
        for (int i = 0; i < 3; i++) Get_Data[i] = 0;
    }
}

void ReadMelder(byte m)
{
    byte Error;
    byte Data;
    byte Data1;
    byte Data2;

    Error = I2c.read(MelderGruppe[m].Adr, 0x01, &Data1);
    delay(2);
    Error += I2c.read(MelderGruppe[m].Adr, 0x01, &Data2);
    if (Data1 == Data2)
    {
        delay(2);
        Error += I2c.read(MelderGruppe[m].Adr, 0x01, &Data1);
        if (Data1 == Data2)
        {
            Data = (0xFF ^ Data2);
            if (Data != MelderGruppe[m].Data)
            {
                MelderGruppe[m].Data = Data;
            }
        }
    }
    if (Error > 0) ErrorZaehler++;
}

void Melde_Melder(byte m)
{
    bool Bit;
    byte Mask = 0x01;
    byte Nr;
    for (byte i = 0; i < 8; i++)
    {
        if ((MelderGruppe[m].Data & Mask))  Bit = true;
        else					        	Bit = false;
        Nr = (8 * m) + i;
        if (Bit != BlockMelder[Nr])
        {
            BlockMelder[Nr] = Bit;
            Send_Data[0] = COM_SEND_BLOCK;
            Send_Data[1] = Nr;
            Send_Data[2] = (byte)Bit;
            Serial.write(Send_Data, COM_LEN(Send_Data[0]));
            delay(100);
        }
        Mask = Mask << 1;
    }
}

void Test_Melder()
{
    if (Block_Test_Nr > 0)
    {
        Send_Data[0] = COM_SEND_BLOCK;
        Send_Data[1] = Block_Test_Nr;
        Send_Data[2] = (byte)true;
        Serial.write(Send_Data, COM_LEN(Send_Data[0]));
        delay(100);
        Send_Data[0] = COM_SEND_BLOCK;
        Send_Data[1] = Block_Test_Nr_alt;
        Send_Data[2] = (byte)false;
        Serial.write(Send_Data, COM_LEN(Send_Data[0]));
        delay(100);
    }
}

byte read_key(void)
{
    uint16_t adcval = analogRead(PIN_KEY);

    if ((adcval >= KEYPAD_KEY_RIGHT_ADC_LOW) && (adcval <= KEYPAD_KEY_RIGHT_ADC_HIGH))   return KEYPAD_KEY_RIGHT;
    if ((adcval >= KEYPAD_KEY_UP_ADC_LOW) && (adcval <= KEYPAD_KEY_UP_ADC_HIGH))         return KEYPAD_KEY_UP;
    if ((adcval >= KEYPAD_KEY_DOWN_ADC_LOW) && (adcval <= KEYPAD_KEY_DOWN_ADC_HIGH))     return KEYPAD_KEY_DOWN;
    if ((adcval >= KEYPAD_KEY_LEFT_ADC_LOW) && (adcval <= KEYPAD_KEY_LEFT_ADC_HIGH))     return KEYPAD_KEY_LEFT;
    if ((adcval >= KEYPAD_KEY_SELECT_ADC_LOW) && (adcval <= KEYPAD_KEY_SELECT_ADC_HIGH)) return KEYPAD_KEY_SELECT;
    if ((adcval >= KEYPAD_KEY_NONE_ADC_LOW) && (adcval <= KEYPAD_KEY_NONE_ADC_HIGH))     return KEYPAD_KEY_NONE;
    return KEYPAD_KEY_NONE;
}


void Wait_Start()
{
    static int Data;
    static byte n = 0;
    static byte Len;

    if (Serial.available() > 0)
    {
        for (;;)
        {
            Data = Serial.read();
            if (n == 0) { Len = COM_LEN(Get_Data[n]); }
            if (n == (Len))
            {
                n = 0;
                break;
            }
            if (Data != -1) n++;
            //TRACE5(Len,"-", n,". bit = ",Data);
        }
    }
}