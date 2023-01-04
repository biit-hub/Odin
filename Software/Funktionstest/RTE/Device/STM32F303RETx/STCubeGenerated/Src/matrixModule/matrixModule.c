
#include "matrixModule.h"




uint8_t row_Port;
uint8_t column_Port;
uint16_t colDelay = 20;	// in ms, Default: 20ms
uint8_t pattern[8];
uint8_t columns = 8;

/*
// nicht invertiert
uint8_t symbols[] = {
0x3F, // 0
0x06, // 1
0x5B, // 2	1101 1010
0x4F, // 3
0x66, // 4
0x6D, // 5
0x7D, // 6
0x07, // 7
0x7F, // 8
0x6F, // 9
0x77, // A
0x7C, // b
0x39, // C
0x5E, // d
0x79, // E
0x71, // F
0x80 // dot
};
*/

// invertiert
uint8_t symbols[] = {
0xc0,
0xf9,
0xa4,
0xb0,
0x99,
0x92,
0x82,
0xf8,
0x80,
0x90,
0x88,
0x83,
0xc6,
0xa1,
0x86,
0x8e,
0x7f
};



bool configHardware(uint8_t colPort, uint8_t rowPort, uint8_t c)	// xx noch unbekanter Datentyp, muss von Odin sein, damit die Pin zuweisung bekannt ist
{
	columns = c;
	if(colPort == rowPort)
	{
		return false; // Ports können nicht gleich sein
	}
	
	if(rowPort > 3 || colPort > 3)
	{
		return false; // Ungültiger Port
	}
	
	// ports setzen
	row_Port = rowPort;
	column_Port = colPort;
}

void setPattern(uint8_t *p)
{
	// *pattern startadresse des Pattern, ab Startadresse 8 Bytes für die werte, jedes Bit im Byte entspricht einer LED
	uint8_t i;
	for(i=0;i<columns;i++)
	{
		pattern[i] = p[i];
	}
}	

// Wird ständig im Main aufgerufen, wechslet automatisch die Spalten	
void showPattern()
{
	// Zeit vergleichen ob abgeleifen ist (Delay zwscihen spalten)
		// Wenn ja, dann spalte inkrementieren
	// spalte anzeige, gemäss Pattern
	
	static uint8_t colCnt = 0;
	static uint32_t lastChange = 0;
	uint32_t timeNow;
	
	if(lastChange + colDelay < timeNow)
	{	
		if(colCnt < columns)
		{
			colCnt++;
		}
		else
		{
			colCnt = 0;
		}
	}
	
	setPort(ROW, 0xFF);
	setPort(COLUMN, 0x01 << colCnt);
	setPort(ROW, pattern[colCnt]);
}

// Zeit für die Leuchtdauer einer Spalte setzen
void setTime(uint16_t time)
{
	colDelay = time;
}

// interne Funktionen
static void setPort(uint8_t mode, uint8_t val)
{
	uint8_t port;
	
	if(mode == ROW)		// eher vertauschen (Column anstatt Row) da Column heikler ist
	{
		port = row_Port;
	}
	else
	{
		port = column_Port;
	}
	
	switch(port)
	{
		case PORT1:
			P1 = val;
		break;
		
		case PORT2:
			P2 = val;
		break;
		
		case PORT3:
			P3 = val << 5;
		break;
	}
}


/*
Handling im main:
- Ständig show Pattern aufrufen (könnte als Thread gelöst werden)
- mit set Pattern, kann das Leuchtmuster gesetzt werden
- Bei z.B. einer Animation, muss auch mit tick auslesen gearbeitet werden


*/