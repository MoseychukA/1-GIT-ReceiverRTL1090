/*
    This example opens Sqlite3 databases from SD Card and
    retrieves data from them.
    Before running please copy following files to SD Card:
    examples/sqlite3_sdmmc/data/mdr512.db
    examples/sqlite3_sdmmc/data/census2000names.db
    Connections:
     * SD Card | ESP32
     *  DAT2       -
     *  DAT3       SS (D5)
     *  CMD        MOSI (D23)
     *  VSS        GND
     *  VDD        3.3V
     *  CLK        SCK (D18)
     *  DAT0       MISO (D19)
     *  DAT1       -
*/
#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>
#include <SPI.h>
#include <FS.h>
#include "SD.h"

const char* data = "Callback function called";
static int callback(void *data, int argc, char **argv, char **azColName){
   int i;
   Serial.printf("%s: ", (const char*)data);
   for (i = 0; i<argc; i++){
       Serial.printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   Serial.printf("\n");
   return 0;
}

int openDb(const char *filename, sqlite3 **db) {
   int rc = sqlite3_open(filename, db);
   if (rc) {
       Serial.printf("Can't open database: %s\n", sqlite3_errmsg(*db));
       return rc;
   } else {
       Serial.printf("Opened database successfully\n");
   }
   return rc;
}

char *zErrMsg = 0;
int db_exec(sqlite3 *db, const char *sql) {
   Serial.println(sql);
   long start = micros();
   int rc = sqlite3_exec(db, sql, callback, (void*)data, &zErrMsg);
   if (rc != SQLITE_OK) {
       Serial.printf("SQL error: %s\n", zErrMsg);
       sqlite3_free(zErrMsg);
   } else {
       Serial.printf("Operation done successfully\n");
   }
   Serial.print(F("Time taken:"));
   Serial.println(micros()-start);
   return rc;
}

#define SOC_GPIO_PIN_MOSI_T5S 15
#define SOC_GPIO_PIN_MISO_T5S 2
#define SOC_GPIO_PIN_SCK_T5S  14
#define SOC_GPIO_PIN_SS_T5S   13

void setup() {
   Serial.begin(38400);
   sqlite3 *db1;
   sqlite3 *db2;
   char *zErrMsg = 0;
   int rc;

   SPI.begin(SOC_GPIO_PIN_SCK_T5S,
             SOC_GPIO_PIN_MISO_T5S,
             SOC_GPIO_PIN_MOSI_T5S,
             SOC_GPIO_PIN_SS_T5S);
   SD.begin();

   sqlite3_initialize();

   // Open database 1
   if (openDb("/sd/fln.db", &db1))
       return;
   if (openDb("/sd/paw.db", &db2))
       return;

   rc = db_exec(db1, "Select * from aircrafts where id = 16222051");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }

   rc = db_exec(db2, "Select * from aircrafts where id = 10624196");
   if (rc != SQLITE_OK) {
       sqlite3_close(db1);
       sqlite3_close(db2);
       return;
   }


   sqlite3_close(db1);
   sqlite3_close(db2);

}

void loop() {
}