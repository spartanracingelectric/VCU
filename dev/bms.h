// ! Fault Thresholds
#define PACK_HIGH_VOLT_FAULT	    410000
#define PACK_LOW_VOLT_FAULT         288000

#define CELL_HIGH_VOLT_FAULT	    43000
#define CELL_LOW_VOLT_FAULT		    30000

#define CELL_HIGH_TEMP_FAULT		60

struct _BMS{
    ubyte2 canMessage;
    ubyte4 highestCellVoltage;
    ubyte2 lowestCellVoltage;
    sbyte2 highestCellTemperature;              
    sbyte2 lowestCellTemperature;

};

ubyte4 BMS_getHighestCellVoltage(BatteryManagementSystem *me);  
ubyte2 BMS_getLowestCellVoltage(BatteryManagementSystem *me);   
sbyte2 BMS_getHighestCellTemp(BatteryManagementSystem* me);  
sbyte2 BMS_getHighestCellTemp(BatteryManagementSystem* me);   