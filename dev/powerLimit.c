/*****************************************************************************
 * powerLimit.c - Power Limiting using a PID controller & LUT to simplify calculations
 * Initial Author(s): Harleen Sandhu/Shaun Gilmore 
 ******************************************************************************
 * Power Limiting code with a flexible Power Target & Initialization Limit
 * Goal: Find a way to limit power under a certain KWH limit (80kwh) while maximizing torque
 * Methods: Currently we are using three methods that are highlighted here:
 *  POWERLIMIT_calculateTorqueCommand: Algorithm is based on using a combination of LUT and the torque equation method
 *  POWERLIMIT_calculateTorqueCommandTorqueEquation: Algorithm is based on a mechanical conversion of power to torque
 *  POWERLIMIT_calculateTorqueCommandPowerPID: Algorithm uses power as a parameter inside the PID the percentage difference of the power is then used to offset torque.  
 * 
 ****************************************************************************/
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file
#include "motorController.h"
#include "PID.h"
//#include "hashTable.h"
#include "powerLimit.h"
#include "mathFunctions.h"

PowerLimit* POWERLIMIT_new(){
    PowerLimit* me = (PowerLimit*)malloc(sizeof(PowerLimit));
    // malloc returns NULL if it fails to allocate memory
    if (me == NULL)
        return NULL;
        
    me->pid = PID_new(40, 20, 0, 231);
    me->plMode = 1;
    /*
    me->hashtable = HashTable_new();
    POWERLIMIT_populateHashTable(me->hashtable, 1);
    
    me->hashtable[5];
    for(ubyte1 mode = 0; mode < 5; ++mode)
    {
        me->hashtable[mode] = *HashTable_new();
        POWERLIMIT_populateHashTable(me->hashtable, mode); 
    }
    */
    me->plStatus = FALSE;
    me->plTorqueCommand = 0; 
    me->plTargetPower = 80;
    me->plInitializationThreshold = me->plTargetPower - 15;

    //LUT Corners
    me->vFloorRFloor = 0;
    me->vFloorRCeiling = 0;
    me->vCeilingRFloor = 0;
    me->vCeilingRCeiling = 0;

    return me;
}

/** SETTER FUNCTIONS  **/

void POWERLIMIT_setModeParameters(PowerLimit* me){

    /* SPACE FOR DASH ROTARY & BUTTON INTERACTIONS */

    /* The below code will be the proper way of interacting with the rotary dial once the button is made. For now, it remains commented out */
    /*
    get the button position and transcribe it to a mode setting

    me->plTargetPower = (9 - me->plMode) * 10;
    me->plInitializationThreshold = me->plTargetPower - 15;
    POWERLIMIT_setLimpModeOverride(PowerLimit* me);

    */

    /* Determine Power Limiting Power Target */
    me->plMode = 9 - (me->plTargetPower / 10); // 9 - 80/10 = 9 - 8 = 1; 9 - 70/10 = 9 - 7 = 2; etc...
    if(me->plTargetPower == 20)
        me->plMode = 5;

    me->plInitializationThreshold = me->plTargetPower - 15;
}

void POWERLIMIT_setLimpModeOverride(PowerLimit* me){
    /*
    if(button press)
        me->plMode = 5;
        me->plTargetPower = 20;
            me->plInitializationThreshold = 0;

    */
}

/** COMPUTATIONS **/

void POWERLIMIT_calculateTorqueCommand(PowerLimit *me, MotorController *mcm){
    
    //if(rotary_button_input != plMode)
    POWERLIMIT_setModeParameters(me);

    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        // Back calculating the hypothetical amount of error that may not belong based on kicking in and out of PL (if not in pl, but has a stored previous error value, does it belong?)
        if(me->plStatus == FALSE){
            PID_setTotalError(me->pid, PID_getTotalError(me->pid)-PID_getPreviousError(me->pid));
        }
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        // Pack Internal Resistance in the VehicleDynamics->power_lim_lut model is 0.027 ohms
        sbyte4 noLoadVoltage = (mcmCurrent * 27 / 1000 ) + mcmVoltage; // 27 / 100 (0.027) is the estimated IR. Should attempt to revalidate on with new powerpack.
        //sbyte4 pidSetpoint = (sbyte4)POWERLIMIT_retrieveTorqueFromLUT(me, &me->hashtable[me->plMode], noLoadVoltage, motorRPM);
        //sbyte2 pidSetpoint = (sbyte2)POWERLIMIT_retrieveTorqueFromLUT(me, me->hashtable, noLoadVoltage, motorRPM);
        
        //issue here
        sbyte2 pidSetpoint = POWERLIMIT_retrieveTorqueFromLUT(me, noLoadVoltage, motorRPM);

        //TQ equation. uncomment to run this instead

        //pidSetpoint = (sbyte2)(me->plTargetPower * 9549 / MCM_getMotorRPM(mcm));

        // If the LUT gives a bad value this is our catch all
        if(pidSetpoint < 0 | pidSetpoint > 231){
            pidSetpoint = (sbyte2)(me->plTargetPower * 9549 / MCM_getMotorRPM(mcm)); 
        }

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);
        
        //TQ equation

        //commandedTorque = (sbyte2)(MCM_getPower(mcm) * 9549 / MCM_getMotorRPM(mcm) / 100);

        //Torque feedback. build later

        //commandedTorque = (sbyte2)(MCM_getTorqueFeedback * 9549 / MCM_getMotorRPM(mcm) / 100);

        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, commandedTorque);
        me->plTorqueCommand = ( commandedTorque + PID_getOutput(me->pid) ) * 10; //deciNewton-meters
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    
    if(POWERLIMIT_getMode(me) >= 20 && POWERLIMIT_getMode(me) < 30){
        POWERLIMIT_calculateTorqueCommandTorqueEquation(me, mcm);
    }

    if(POWERLIMIT_getMode(me) >= 30 && POWERLIMIT_getMode(me) < 40){
        POWERLIMIT_calculateTorqueCommandPowerPID(me, mcm);
    }
}

sbyte2 POWERLIMIT_retrieveTorqueFromLUT(PowerLimit *me, sbyte4 voltage, sbyte4 rpm){    // Find the floor and ceiling values for voltage and rpm
    
    // LUT Lower Bounds
    ubyte4 VOLTAGE_MIN      = 280;
    ubyte4 RPM_MIN          = 2000;
    
    // Calculating hashtable keys
    ubyte4 rpmInput         = (ubyte4)rpm - RPM_MIN;
    ubyte4 voltageInput     = (ubyte4)voltage - VOLTAGE_MIN;
    ubyte4 voltageFloor     = ubyte4_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 voltageCeiling   = ubyte4_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    ubyte4 rpmFloor         = ubyte4_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    ubyte4 rpmCeiling       = ubyte4_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    ubyte4 voltageLowerDiff = voltage - voltageFloor;
    ubyte4 voltageUpperDiff = voltageCeiling - voltage;
    ubyte4 rpmLowerDiff     = rpm - rpmFloor;
    ubyte4 rpmUpperDiff     = rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    me->vFloorRFloor     = POWERLIMIT_getTorqueFromArray(voltageFloor, rpmFloor);
    me->vFloorRCeiling   = POWERLIMIT_getTorqueFromArray(voltageFloor, rpmCeiling);
    me->vCeilingRFloor   = POWERLIMIT_getTorqueFromArray(voltageCeiling, rpmFloor);
    me->vCeilingRCeiling = POWERLIMIT_getTorqueFromArray(voltageCeiling, rpmCeiling);

    // If voltageFloor == voltageCeiling then voltageLowerDiff == voltageUpperDiff == 0, which means we get a multiply by 0 error.
    // We want a single interpolation bypass for any of these scenarios

    if(voltageLowerDiff == 0){
        return (sbyte2) ((ubyte4) me->vFloorRFloor + (rpmLowerDiff) * (me->vFloorRCeiling - me->vFloorRFloor) / RPM_STEP);
    }

    if(rpmLowerDiff == 0){
        return (sbyte2) ((ubyte4) me->vFloorRFloor + (voltageLowerDiff) * (me->vCeilingRFloor - me->vFloorRFloor) / VOLTAGE_STEP);
    }

    // Calculate interpolation values
    ubyte4 stepDivider          = (ubyte4)VOLTAGE_STEP          * RPM_STEP;
    ubyte4 torqueFloorFloor     = (ubyte4)me->vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    ubyte4 torqueFloorCeiling   = (ubyte4)me->vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    ubyte4 torqueCeilingFloor   = (ubyte4)me->vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    ubyte4 torqueCeilingCeiling = (ubyte4)me->vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    return (sbyte2)((torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider);
}

void POWERLIMIT_calculateTorqueCommandTorqueEquation(PowerLimit *me, MotorController *mcm){
    //doing this should be illegal, but since pl mode is also going to be used for the equation version for right now, i feel fine about it. 2 for second pl method, 1 representing the pwoer target
    me->plMode = 21;
    PID_setSaturationPoint(me->pid, 8000);
    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);

        sbyte2 pidSetpoint = (sbyte2)((sbyte4)me->plTargetPower * 9549 / MCM_getMotorRPM(mcm));

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(me->pid, pidSetpoint);
        PID_computeOutput(me->pid, commandedTorque);
        me->plTorqueCommand = ( commandedTorque + PID_getOutput(me->pid) ) * 10; //deciNewton-meters
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}

void POWERLIMIT_calculateTorqueCommandPowerPID(PowerLimit *me, MotorController *mcm){
        //doing this should be illegal, but since pl mode is also going to be used for the equation version for right now, i feel fine about it. 3 for third pl method, 1 representing the pwoer target
    PID_setSaturationPoint(me->pid, 8000);
    me->plMode = 31;
    if( (MCM_getPower(mcm) / 1000) > me->plInitializationThreshold){
        me->plStatus = TRUE;

        /* Sensor inputs */
        sbyte4 motorRPM   = MCM_getMotorRPM(mcm);
        sbyte4 mcmVoltage = MCM_getDCVoltage(mcm);
        sbyte4 mcmCurrent = MCM_getDCCurrent(mcm);

        sbyte2 pidTargetValue = me->plTargetPower * 100;
        sbyte2 pidCurrentValue = (sbyte2) MCM_getPower(mcm) / 10;

        sbyte2 commandedTorque = (sbyte2)MCM_getCommandedTorque(mcm);

        PID_updateSetpoint(me->pid, pidTargetValue);
        PID_computeOutput(me->pid, pidCurrentValue);
        me->plTorqueCommand = (sbyte2) ((sbyte4) commandedTorque + commandedTorque * PID_getOutput(me->pid) / pidCurrentValue) * 10; //deciNewton-meters
        if(me->plTorqueCommand > MCM_MAX_TORQUE_DNm)
            me->plTorqueCommand = MCM_MAX_TORQUE_DNm;
            
        MCM_update_PL_setTorqueCommand(mcm, me->plTorqueCommand);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
    else {
        me->plStatus = FALSE;
        MCM_update_PL_setTorqueCommand(mcm, -1);
        MCM_set_PL_updateStatus(mcm, me->plStatus);
    }
}


/*
void POWERLIMIT_populateHashTable(HashTable* table, ubyte1 target)
{
    /*
    ubyte2 VOLTAGE_MIN = 280;
    ubyte2 VOLTAGE_MAX = 405;
    ubyte2 RPM_MIN = 2000; // Data analysis says 2340 rpm min @ 70kW, on oct7-8 launch for sr-14
    ubyte2 RPM_MAX = 6000;
    const ubyte1 NUM_V = 26;
    const ubyte1 NUM_S = 26;
    ubyte2 (*pointer)[NUM_V];
    // cases:
    // 1 - 80 kW
    // 2 - 70 kW
    // 3 - 60 kW
    // 4 - 50 kW
    // 5 - 20 kW LIMP MODE
    switch(target){
        case 1:
            VOLTAGE_MIN = 280;
            VOLTAGE_MAX = 405;
            RPM_MIN = 2000;
            RPM_MAX = 6000;
            const ubyte2 POWER_LIM_LUT_80[26][26] = {
                {2357, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {2265, 2299, 2343, 2385, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {2039, 2147, 2218, 2288, 2332, 2372, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {1880, 1964, 2055, 2146, 2224, 2270, 2321, 2360, 2395, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {1722, 1804, 1855, 1966, 2039, 2132, 2223, 2269, 2312, 2348, 2388, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {1560, 1642, 1731, 1819, 1896, 1968, 2038, 2131, 2209, 2268, 2310, 2392, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {1426, 1509, 1589, 1669, 1754, 1832, 1908, 1970, 2038, 2129, 2209, 2318, 2386, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399, 2399},
                {1308, 1379, 1466, 1553, 1609, 1701, 1777, 1851, 1915, 2001, 2073, 2224, 2317, 2373, 2377, 2380, 2381, 2382, 2383, 2385, 2384, 2384, 2384, 2384, 2384, 2384},
                {1177, 1261, 1343, 1425, 1495, 1577, 1644, 1716, 1789, 1855, 1927, 2106, 2211, 2265, 2265, 2268, 2270, 2271, 2272, 2272, 2273, 2273, 2273, 2273, 2273, 2273},
                {1065, 1149, 1229, 1308, 1370, 1445, 1530, 1595, 1669, 1720, 1805, 1964, 2107, 2160, 2163, 2166, 2167, 2169, 2170, 2171, 2171, 2172, 2172, 2172, 2172, 2172},
                {951, 1036, 1117, 1197, 1263, 1341, 1413, 1472, 1554, 1621, 1699, 1854, 1971, 2058, 2070, 2072, 2075, 2076, 2077, 2078, 2079, 2079, 2080, 2078, 2078, 2078},
                {839, 926, 1008, 1090, 1167, 1238, 1308, 1377, 1431, 1508, 1589, 1747, 1861, 1954, 1984, 1986, 1988, 1990, 1991, 1992, 1993, 1993, 1994, 1994, 1994, 1994},
                {726, 818, 898, 977, 1058, 1133, 1193, 1265, 1340, 1398, 1469, 1633, 1764, 1858, 1904, 1907, 1908, 1911, 1912, 1913, 1914, 1915, 1914, 1915, 1915, 1916},
                {611, 708, 794, 880, 959, 1035, 1103, 1174, 1232, 1308, 1382, 1539, 1661, 1777, 1823, 1833, 1864, 1837, 1839, 1839, 1840, 1841, 1843, 1842, 1842, 1842},
                {481, 595, 687, 778, 857, 934, 1007, 1077, 1144, 1209, 1276, 1427, 1565, 1675, 1746, 1765, 1767, 1770, 1770, 1770, 1772, 1773, 1774, 1774, 1774, 1775},
                {337, 472, 572, 672, 760, 838, 913, 987, 1056, 1118, 1192, 1343, 1459, 1588, 1667, 1697, 1703, 1705, 1706, 1708, 1707, 1710, 1710, 1711, 1711, 1712},
                {119, 325, 446, 566, 657, 739, 819, 893, 963, 1024, 1103, 1248, 1384, 1492, 1592, 1631, 1644, 1647, 1648, 1649, 1650, 1650, 1651, 1652, 1652, 1653},
                {1, 102, 314, 445, 550, 641, 725, 803, 873, 939, 1011, 1173, 1294, 1399, 1502, 1577, 1590, 1591, 1593, 1593, 1594, 1596, 1596, 1596, 1598, 1598},
                {1, 1,  82, 303, 432, 537, 626, 706, 783, 853, 932, 1085, 1210, 1315, 1419, 1508, 1531, 1539, 1540, 1541, 1543, 1544, 1545, 1545, 1546, 1546},
                {1, 1, 1, 61, 290, 421, 524, 614, 693, 769, 849, 1006, 1134, 1234, 1340, 1428, 1477, 1490, 1493, 1493, 1495, 1496, 1497, 1493, 1497, 1498},
                {1, 1, 1, 1, 35, 279, 410, 511, 600, 679, 763, 926, 1057, 1168, 1264, 1351, 1431, 1442, 1447, 1448, 1449, 1450, 1450, 1463, 1451, 1452},
                {1, 1, 1, 1, 1, 1, 270, 397, 500, 586, 675, 849, 980, 1085, 1189, 1266, 1358, 1393, 1403, 1404, 1405, 1406, 1407, 1408, 1408, 1409},
                {1, 1, 1, 1, 1, 1, 1, 258, 388, 489, 588, 769, 899, 1015, 1113, 1202, 1276, 1346, 1361, 1364, 1365, 1365, 1367, 1367, 1368, 1369},
                {1, 1, 1, 1, 1, 1, 1, 1, 247, 378, 491, 687, 826, 939, 1039, 1132, 1210, 1291, 1317, 1335, 1327, 1327, 1328, 1328, 1330, 1330},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 238, 383, 603, 751, 869, 971, 1059, 1142, 1217, 1280, 1292, 1290, 1291, 1292, 1293, 1292, 1294},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 249, 514, 676, 796, 901, 993, 1074, 1152, 1221, 1246, 1256, 1258, 1257, 1258, 1259, 1259}};
            pointer = &POWER_LIM_LUT_80;
        break;

        case 2: // 70 kW
            VOLTAGE_MIN = 280;
            VOLTAGE_MAX = 405;
            RPM_MIN = 2000;
            RPM_MAX = 6000;
            const ubyte2 POWER_LIM_LUT_70[26][26] = {
                {2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {2216, 2294, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {2053, 2141, 2215, 2280, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {1870, 1977, 2052, 2140, 2214, 2281, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {1723, 1804, 1888, 1982, 2052, 2141, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {1573, 1658, 1738, 1821, 1892, 1998, 2214, 2306, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {1437, 1521, 1583, 1683, 1753, 1845, 2067, 2183, 2190, 2194, 2196, 2199, 2198, 2201, 2201, 2201, 2201, 2201, 2201, 2201, 2201, 2201, 2201, 2201, 2201, 2201},
                {1308, 1377, 1474, 1536, 1611, 1714, 1922, 2056, 2082, 2085, 2088, 2089, 2091, 2095, 2092, 2092, 2092, 2092, 2092, 2092, 2092, 2092, 2092, 2092, 2092, 2092},
                {1183, 1259, 1347, 1415, 1500, 1586, 1793, 1947, 1983, 1987, 1990, 1991, 1992, 1993, 1991, 1994, 1994, 1995, 1995, 1995, 1995, 1995, 1995, 1995, 1995, 1995},
                {1064, 1140, 1221, 1308, 1383, 1474, 1685, 1838, 1893, 1896, 1899, 1907, 1903, 1904, 1904, 1904, 1904, 1905, 1905, 1905, 1905, 1905, 1905, 1905, 1905, 1905},
                {946, 1036, 1118, 1195, 1273, 1356, 1567, 1727, 1798, 1814, 1816, 1818, 1819, 1821, 1822, 1823, 1823, 1823, 1824, 1824, 1824, 1824, 1824, 1824, 1824, 1824},
                {839, 927, 1008, 1087, 1164, 1248, 1461, 1614, 1719, 1738, 1741, 1738, 1744, 1745, 1747, 1748, 1748, 1749, 1749, 1749, 1748, 1748, 1748, 1748, 1748, 1748},
                {727, 813, 902, 976, 1057, 1149, 1353, 1510, 1639, 1668, 1668, 1673, 1673, 1676, 1676, 1678, 1679, 1678, 1679, 1680, 1680, 1680, 1680, 1680, 1680, 1680},
                {610, 707, 797, 880, 953, 1052, 1257, 1401, 1536, 1597, 1606, 1607, 1610, 1610, 1612, 1613, 1614, 1615, 1614, 1615, 1616, 1615, 1615, 1615, 1615, 1615},
                {484, 594, 690, 773, 858, 953, 1164, 1317, 1440, 1524, 1546, 1548, 1549, 1551, 1552, 1553, 1554, 1554, 1555, 1555, 1556, 1556, 1556, 1556, 1556, 1556},
                {339, 472, 578, 673, 757, 856, 1070, 1217, 1348, 1460, 1487, 1492, 1494, 1495, 1496, 1497, 1498, 1499, 1499, 1500, 1500, 1500, 1501, 1501, 1512, 1501},
                {119, 326, 457, 564, 657, 757, 980, 1134, 1262, 1373, 1426, 1440, 1441, 1443, 1443, 1445, 1446, 1446, 1446, 1450, 1449, 1448, 1449, 1449, 1449, 1449},
                {1, 102, 313, 445, 550, 660, 892, 1043, 1173, 1277, 1379, 1390, 1392, 1394, 1395, 1396, 1401, 1398, 1399, 1399, 1396, 1401, 1401, 1401, 1401, 1401},
                {1, 1, 83, 302, 432, 558, 802, 965, 1093, 1205, 1302, 1336, 1347, 1348, 1349, 1350, 1351, 1352, 1353, 1354, 1355, 1355, 1355, 1355, 1355, 1356},
                {1, 1, 1, 61, 290, 445, 711, 880, 1012, 1126, 1214, 1288, 1305, 1306, 1307, 1308, 1308, 1310, 1311, 1311, 1313, 1312, 1313, 1312, 1313, 1314},
                {1, 1, 1, 1, 36, 313, 623, 797, 931, 1043, 1146, 1236, 1260, 1266, 1266, 1267, 1269, 1270, 1271, 1270, 1272, 1271, 1273, 1273, 1273, 1274},
                {1, 1, 1, 1, 1, 108, 522, 711, 852, 967, 1065, 1159, 1211, 1229, 1230, 1230, 1231, 1232, 1232, 1233, 1234, 1234, 1235, 1226, 1235, 1235},
                {1, 1, 1, 1, 1, 1, 415, 625, 772, 892, 995, 1084, 1166, 1189, 1194, 1193, 1196, 1196, 1197, 1198, 1197, 1199, 1199, 1200, 1200, 1200},
                {1, 1, 1, 1, 1, 1, 286, 534, 691, 814, 918, 1013, 1095, 1151, 1159, 1160, 1162, 1161, 1164, 1164, 1165, 1165, 1165, 1165, 1166, 1166},
                {1, 1, 1, 1, 1, 1, 68, 434, 607, 738, 845, 941, 1026, 1103, 1124, 1124, 1130, 1130, 1131, 1131, 1132, 1132, 1134, 1133, 1134, 1135},
                {1, 1, 1, 1, 1, 1, 1, 315, 517, 659, 772, 870, 958, 1034, 1089, 1098, 1099, 1100, 1100, 1101, 1102, 1102, 1103, 1103, 1103, 1104}};
            pointer = &POWER_LIM_LUT_70;
        break;
        
        case 3: //60 kW
            VOLTAGE_MIN = 280;
            VOLTAGE_MAX = 405;
            RPM_MIN = 2000;
            RPM_MAX = 6000;
            const ubyte2 POWER_LIM_LUT_60[26][26] = {
                {2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {2257, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {2084, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
                {1912, 2208, 2228, 2234, 2237, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238, 2238},
                {1754, 2061, 2098, 2104, 2108, 2108, 2110, 2110, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109, 2109},
                {1596, 1918, 1983, 1987, 1992, 1993, 1994, 1995, 1994, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996},
                {1475, 1783, 1878, 1883, 1887, 1889, 1891, 1892, 1892, 1894, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890, 1890},
                {1331, 1643, 1773, 1789, 1792, 1796, 1799, 1798, 1799, 1799, 1798, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799, 1799},
                {1216, 1513, 1682, 1704, 1707, 1709, 1711, 1713, 1714, 1714, 1714, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712, 1712},
                {1102, 1394, 1585, 1625, 1629, 1631, 1632, 1635, 1636, 1636, 1638, 1638, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637, 1637},
                {992, 1292, 1465, 1552, 1556, 1559, 1561, 1562, 1564, 1565, 1566, 1564, 1567, 1568, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567, 1567},
                {879, 1184, 1367, 1477, 1491, 1494, 1496, 1497, 1499, 1500, 1501, 1503, 1501, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502},
                {763, 1079, 1265, 1399, 1431, 1433, 1432, 1437, 1445, 1439, 1440, 1441, 1441, 1442, 1442, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443, 1443},
                {654, 980, 1165, 1303, 1369, 1376, 1379, 1381, 1382, 1383, 1384, 1385, 1386, 1386, 1387, 1387, 1387, 1388, 1387, 1387, 1387, 1387, 1387, 1387, 1387, 1387},
                {536, 881, 1070, 1205, 1316, 1325, 1328, 1329, 1332, 1332, 1333, 1334, 1334, 1334, 1335, 1335, 1336, 1337, 1336, 1337, 1337, 1337, 1337, 1337, 1337, 1337},
                {403, 783, 975, 1126, 1246, 1274, 1280, 1281, 1282, 1284, 1285, 1286, 1285, 1286, 1287, 1287, 1287, 1288, 1286, 1289, 1289, 1289, 1289, 1289, 1289, 1289},
                {231, 684, 885, 1032, 1158, 1226, 1235, 1236, 1237, 1239, 1239, 1239, 1241, 1241, 1242, 1243, 1243, 1244, 1243, 1243, 1244, 1245, 1247, 1247, 1247, 1247},
                {1, 579, 793, 946, 1070, 1177, 1192, 1185, 1196, 1196, 1197, 1199, 1200, 1200, 1202, 1200, 1202, 1203, 1202, 1203, 1202, 1203, 1202, 1202, 1202, 1202},
                {1, 466, 700, 858, 987, 1095, 1149, 1156, 1156, 1157, 1158, 1164, 1160, 1161, 1162, 1161, 1163, 1163, 1161, 1164, 1163, 1164, 1164, 1164, 1164, 1165},
                {1, 335, 603, 772, 902, 1014, 1109, 1116, 1120, 1120, 1121, 1122, 1123, 1123, 1124, 1125, 1125, 1126, 1127, 1126, 1127, 1127, 1127, 1127, 1127, 1128},
                {1, 147, 500, 684, 816, 935, 1033, 1075, 1084, 1085, 1087, 1087, 1089, 1089, 1090, 1090, 1090, 1092, 1089, 1092, 1094, 1093, 1098, 1093, 1094, 1092},
                {1, 1, 384, 591, 736, 855, 959, 1040, 1051, 1053, 1054, 1056, 1056, 1056, 1057, 1058, 1058, 1058, 1059, 1059, 1060, 1060, 1060, 1061, 1061, 1061},
                {1, 1, 239, 493, 650, 776, 880, 973, 1016, 1022, 1023, 1024, 1026, 1026, 1026, 1027, 1027, 1028, 1028, 1028, 1029, 1028, 1030, 1030, 1030, 1030},
                {1, 1, 1, 383, 562, 693, 805, 901, 982, 991, 994, 995, 996, 998, 997, 998, 999, 1000, 1000, 1003, 999, 1001, 1001, 1001, 1001, 1000},
                {1, 1, 1, 244, 466, 611, 727, 825, 913, 960, 967, 968, 968, 969, 970, 970, 971, 971, 971, 972, 973, 973, 973, 974, 953, 975},
                {1, 1, 1, 1, 356, 522, 649, 753, 841, 927, 938, 942, 942, 943, 944, 944, 945, 942, 946, 946, 935, 949, 947, 948, 952, 948}};
            pointer = &POWER_LIM_LUT_60;
        break;
        
        case 4: //50 kW
            VOLTAGE_MIN = 280;
            VOLTAGE_MAX = 405;
            RPM_MIN = 2000;
            RPM_MAX = 6000;
            const ubyte2 POWER_LIM_LUT_50[26][26] = {
                {2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293, 2293},
                {2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134, 2134},
                {1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996, 1996},
                {1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874, 1874},
                {1766, 1766, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767, 1767},
                {1672, 1668, 1669, 1669, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678, 1678},
                {1579, 1581, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582, 1582},
                {1500, 1502, 1503, 1505, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504, 1504},
                {1428, 1429, 1431, 1433, 1432, 1433, 1433, 1433, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434, 1434},
                {1362, 1364, 1365, 1367, 1365, 1368, 1368, 1367, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368, 1368},
                {1302, 1304, 1304, 1305, 1307, 1307, 1306, 1308, 1308, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309, 1309},
                {1246, 1249, 1250, 1250, 1252, 1252, 1253, 1254, 1255, 1254, 1252, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254, 1254},
                {1195, 1198, 1199, 1200, 1201, 1202, 1203, 1203, 1203, 1204, 1204, 1205, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204, 1204},
                {1148, 1150, 1151, 1153, 1154, 1155, 1155, 1157, 1157, 1158, 1161, 1159, 1158, 1159, 1159, 1159, 1159, 1159, 1159, 1159, 1159, 1159, 1159, 1159, 1159, 1159},
                {1105, 1106, 1108, 1109, 1110, 1112, 1111, 1116, 1113, 1114, 1115, 1115, 1116, 1115, 1116, 1116, 1116, 1116, 1116, 1116, 1116, 1116, 1116, 1116, 1116, 1116},
                {1062, 1067, 1067, 1069, 1070, 1070, 1071, 1072, 1073, 1074, 1075, 1075, 1075, 1076, 1075, 1076, 1074, 1075, 1075, 1075, 1075, 1075, 1075, 1075, 1075, 1075},
                {1021, 1029, 1030, 1031, 1032, 1033, 1034, 1035, 1036, 1036, 1048, 1037, 1037, 1038, 1038, 1038, 1038, 1038, 1039, 1039, 1039, 1039, 1039, 1039, 1039, 1039},
                {980, 992, 995, 996, 997, 997, 999, 1000, 1000, 1001, 1001, 1002, 1002, 1003, 1002, 1003, 1003, 1004, 1004, 1004, 1004, 1004, 1004, 1004, 1004, 1004},
                {894, 956, 962, 963, 964, 965, 966, 967, 968, 968, 968, 969, 969, 969, 970, 971, 971, 971, 971, 971, 971, 972, 972, 972, 972, 972},
                {809, 918, 930, 932, 933, 934, 934, 936, 936, 937, 938, 975, 939, 939, 939, 939, 940, 940, 942, 941, 941, 941, 940, 941, 941, 941},
                {721, 836, 896, 903, 904, 905, 906, 907, 908, 908, 909, 909, 910, 910, 910, 911, 910, 910, 912, 912, 912, 912, 912, 912, 911, 914},
                {633, 754, 857, 875, 877, 878, 879, 879, 880, 881, 881, 882, 882, 883, 883, 884, 884, 884, 885, 884, 885, 885, 885, 885, 887, 886},
                {540, 670, 778, 845, 852, 864, 853, 854, 854, 855, 856, 856, 857, 857, 857, 858, 858, 858, 859, 859, 859, 859, 860, 860, 860, 859},
                {438, 582, 697, 796, 824, 828, 829, 829, 830, 831, 831, 832, 832, 836, 833, 834, 834, 834, 834, 835, 835, 835, 835, 837, 836, 837},
                {318, 488, 615, 717, 801, 803, 806, 807, 807, 807, 808, 809, 810, 810, 810, 810, 810, 811, 811, 811, 812, 813, 812, 812, 812, 813},
                {150, 383, 526, 638, 735, 779, 783, 785, 785, 785, 787, 786, 788, 787, 793, 789, 789, 789, 790, 790, 790, 791, 790, 791, 791, 792}};
            pointer = &POWER_LIM_LUT_50;
        break;


        case 5:
            VOLTAGE_MIN = 280;
            VOLTAGE_MAX = 405;
            RPM_MIN = 2000;
            RPM_MAX = 6000;
            const ubyte2 POWER_LIM_LUT_20[26][26] = {
                {940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940},
                {872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872, 872},
                {813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813, 813},
                {762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762, 762},
                {717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717, 717},
                {676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676, 676},
                {641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641, 641},
                {608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608, 608},
                {578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578, 578},
                {552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552, 552},
                {528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528, 528},
                {505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505, 505},
                {485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485, 485},
                {466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466, 466},
                {449, 448, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449, 449},
                {432, 432, 432, 433, 432, 432, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433, 433},
                {415, 415, 416, 417, 417, 417, 417, 419, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418, 418},
                {408, 402, 402, 402, 403, 403, 403, 400, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403, 403},
                {388, 388, 389, 389, 389, 389, 390, 390, 390, 391, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390, 390},
                {375, 376, 376, 376, 377, 377, 377, 377, 376, 378, 378, 378, 378, 377, 378, 378, 378, 378, 378, 378, 378, 378, 378, 378, 378, 378},
                {364, 364, 365, 364, 364, 365, 365, 369, 366, 365, 366, 365, 366, 366, 366, 366, 366, 366, 366, 366, 366, 366, 366, 366, 366, 366},
                {352, 352, 353, 353, 353, 354, 354, 354, 354, 355, 355, 355, 355, 355, 355, 355, 355, 355, 354, 357, 357, 357, 357, 357, 357, 357},
                {342, 341, 344, 342, 342, 343, 343, 343, 344, 344, 345, 344, 345, 344, 345, 345, 345, 345, 346, 346, 346, 346, 346, 346, 346, 346},
                {332, 359, 332, 332, 332, 334, 334, 333, 334, 334, 335, 334, 334, 335, 335, 336, 336, 335, 335, 335, 336, 333, 333, 338, 338, 338},
                {321, 322, 323, 323, 323, 324, 323, 324, 324, 325, 325, 325, 322, 326, 325, 325, 326, 326, 326, 327, 327, 325, 326, 326, 326, 326},
                {312, 313, 313, 314, 315, 314, 315, 315, 315, 316, 316, 316, 317, 316, 317, 316, 317, 317, 318, 318, 317, 317, 318, 318, 318, 316}};
            pointer = &POWER_LIM_LUT_20;
        break;

        default:
        break;
    }
    
    ubyte2 VOLTAGE_MIN = 280;
    ubyte2 VOLTAGE_MAX = 405;
    ubyte2 RPM_MIN = 2000; // Data analysis says 2340 rpm min @ 70kW, on oct7-8 launch for sr-14
    ubyte2 RPM_MAX = 6000;
    const ubyte1 NUM_V = 26;
    const ubyte1 NUM_S = 26;

    const ubyte1 POWER_LIM_LUT_80[26][26] = {
        {231, 231, 199, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {222, 229, 180, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {205, 214, 161, 228, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {187, 198, 146, 214, 221, 227, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {172, 180, 130, 198, 205, 214, 221, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {157, 166, 117, 182, 189, 198, 205, 213, 218, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {144, 152, 103, 168, 175, 183, 190, 199, 205, 213, 218, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {131, 138, 90, 154, 161, 170, 177, 184, 193, 199, 207, 221, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {118, 126, 77, 141, 150, 157, 164, 171, 179, 185, 192, 208, 221, 226, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227},
        {106, 114, 63, 131, 138, 145, 153, 159, 167, 173, 180, 198, 209, 216, 216, 217, 217, 217, 217, 216, 217, 217, 217, 217, 217, 217},
        {95, 104, 48, 120, 127, 133, 140, 148, 155, 162, 169, 185, 199, 205, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 208},
        {84, 93, 32, 109, 116, 123, 131, 137, 144, 151, 157, 174, 186, 197, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199},
        {73, 81, 0, 98, 106, 113, 121, 127, 134, 139, 147, 164, 176, 186, 190, 191, 191, 191, 191, 191, 191, 192, 191, 191, 192, 192},
        {61, 71, 0, 88, 95, 103, 110, 117, 124, 131, 138, 153, 166, 177, 182, 183, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184},
        {48, 59, 0, 77, 86, 94, 101, 108, 115, 120, 128, 144, 157, 166, 175, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177},
        {34, 47, 0, 67, 76, 84, 91, 99, 105, 111, 119, 135, 147, 157, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 171, 171},
        {12, 33, 0, 56, 66, 74, 82, 89, 96, 103, 110, 126, 138, 150, 159, 164, 164, 165, 165, 165, 165, 165, 165, 165, 165, 165},
        {0, 10, 0, 44, 55, 64, 72, 80, 88, 94, 102, 117, 129, 139, 150, 156, 159, 159, 159, 159, 159, 160, 160, 160, 160, 160},
        {0, 0, 0, 30, 43, 54, 63, 71, 78, 86, 93, 109, 120, 132, 141, 151, 153, 154, 154, 154, 154, 154, 154, 154, 155, 155},
        {0, 0, 0, 6, 29, 42, 52, 61, 69, 77, 85, 101, 113, 125, 134, 142, 148, 149, 149, 150, 149, 150, 150, 150, 150, 150},
        {0, 0, 0, 0, 4, 28, 41, 51, 60, 68, 76, 93, 105, 117, 126, 135, 142, 144, 145, 145, 145, 145, 145, 145, 145, 142},
        {0, 0, 0, 0, 0, 0, 27, 40, 50, 59, 68, 85, 98, 109, 119, 127, 136, 139, 140, 140, 141, 141, 141, 141, 141, 141},
        {0, 0, 0, 0, 0, 0, 0, 26, 39, 49, 59, 77, 90, 101, 111, 120, 128, 134, 136, 136, 137, 137, 137, 137, 137, 137},
        {0, 0, 0, 0, 0, 0, 0, 0, 25, 38, 49, 68, 83, 94, 104, 113, 121, 129, 132, 132, 133, 133, 133, 133, 133, 133},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 38, 60, 75, 87, 97, 106, 115, 121, 127, 129, 129, 129, 129, 129, 129, 129},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 51, 67, 80, 90, 99, 107, 115, 122, 125, 126, 126, 126, 126, 126, 126}};

    for(ubyte1 row = 0; row < NUM_S; ++row) {
        for(ubyte1 column = 0; column < NUM_V; ++column) {
            ubyte2 voltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            ubyte2 rpm   = RPM_MIN + row * RPM_STEP;
            ubyte1 value = POWER_LIM_LUT_80[row][column];
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }
    /*
    for(ubyte1 row = 0; row < NUM_S; ++row) {
        for(ubyte1 column = 0; column < NUM_V; ++column) {
            ubyte2 voltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            ubyte2 rpm   = RPM_MIN + row * RPM_STEP;
            ubyte2 value = *(*(pointer + row) + column);
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }
    
}
*/

/** GETTER FUNCTIONS **/

ubyte1 POWERLIMIT_getStatusCodeBlock(PowerLimit* me){
    ubyte1 eightBits = 0x00;
    //First Bit
    if(me->plStatus)
        eightBits | (1 << 7);
    // 2nd through 4th bit
    eightBits | (me->plMode << 4);
    // 5th bit
    eightBits | (PID_getAntiWindupFlag(me->pid) << 3);
    // 6th bit and beyond
    
    return eightBits;
}

bool POWERLIMIT_getStatus(PowerLimit* me){
    return me->plStatus;
}

ubyte1 POWERLIMIT_getMode(PowerLimit* me){
    return me->plMode;
}

sbyte2 POWERLIMIT_getTorqueCommand(PowerLimit* me){
    return me->plTorqueCommand;
}

ubyte1 POWERLIMIT_getTargetPower(PowerLimit* me){
    return me->plTargetPower;
}

ubyte1 POWERLIMIT_getInitialisationThreshold(PowerLimit* me){
    return me->plInitializationThreshold;
}

ubyte1 POWERLIMIT_getLUTCorner(PowerLimit* me, ubyte1 corner){
    // corner cases:
    // 1 - lowerX lowerY
    // 2 - lowerX lowerY
    // 3 - higherX lowerY
    // 4 - higherX higherY
    switch(corner){
        case 1:
            return me->vFloorRFloor;

        case 2:
            return me->vFloorRCeiling;
        
        case 3:
            return me->vCeilingRFloor;
        
        case 4:
            return me->vCeilingRCeiling;
        
        default:
            return 0xFF;
    }
}

ubyte1 POWERLIMIT_getTorqueFromArray(ubyte4 noLoadVoltage, ubyte4 rpm)
{
    ubyte2 VOLTAGE_MIN = 280;
    ubyte2 VOLTAGE_MAX = 405;
    ubyte2 RPM_MIN = 2000; // Data analysis says 2340 rpm min @ 70kW, on oct7-8 launch for sr-14
    ubyte2 RPM_MAX = 6000;
    const ubyte1 NUM_V = 26;
    const ubyte1 NUM_S = 26;

    const ubyte1 POWER_LIM_LUT_80[26][26] = {
        {231, 231, 199, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {222, 229, 180, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {205, 214, 161, 228, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {187, 198, 146, 214, 221, 227, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {172, 180, 130, 198, 205, 214, 221, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {157, 166, 117, 182, 189, 198, 205, 213, 218, 226, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {144, 152, 103, 168, 175, 183, 190, 199, 205, 213, 218, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {131, 138, 90, 154, 161, 170, 177, 184, 193, 199, 207, 221, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231},
        {118, 126, 77, 141, 150, 157, 164, 171, 179, 185, 192, 208, 221, 226, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227},
        {106, 114, 63, 131, 138, 145, 153, 159, 167, 173, 180, 198, 209, 216, 216, 217, 217, 217, 217, 216, 217, 217, 217, 217, 217, 217},
        {95, 104, 48, 120, 127, 133, 140, 148, 155, 162, 169, 185, 199, 205, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 208},
        {84, 93, 32, 109, 116, 123, 131, 137, 144, 151, 157, 174, 186, 197, 198, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199, 199},
        {73, 81, 0, 98, 106, 113, 121, 127, 134, 139, 147, 164, 176, 186, 190, 191, 191, 191, 191, 191, 191, 192, 191, 191, 192, 192},
        {61, 71, 0, 88, 95, 103, 110, 117, 124, 131, 138, 153, 166, 177, 182, 183, 184, 184, 184, 184, 184, 184, 184, 184, 184, 184},
        {48, 59, 0, 77, 86, 94, 101, 108, 115, 120, 128, 144, 157, 166, 175, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177, 177},
        {34, 47, 0, 67, 76, 84, 91, 99, 105, 111, 119, 135, 147, 157, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 171, 171},
        {12, 33, 0, 56, 66, 74, 82, 89, 96, 103, 110, 126, 138, 150, 159, 164, 164, 165, 165, 165, 165, 165, 165, 165, 165, 165},
        {0, 10, 0, 44, 55, 64, 72, 80, 88, 94, 102, 117, 129, 139, 150, 156, 159, 159, 159, 159, 159, 160, 160, 160, 160, 160},
        {0, 0, 0, 30, 43, 54, 63, 71, 78, 86, 93, 109, 120, 132, 141, 151, 153, 154, 154, 154, 154, 154, 154, 154, 155, 155},
        {0, 0, 0, 6, 29, 42, 52, 61, 69, 77, 85, 101, 113, 125, 134, 142, 148, 149, 149, 150, 149, 150, 150, 150, 150, 150},
        {0, 0, 0, 0, 4, 28, 41, 51, 60, 68, 76, 93, 105, 117, 126, 135, 142, 144, 145, 145, 145, 145, 145, 145, 145, 142},
        {0, 0, 0, 0, 0, 0, 27, 40, 50, 59, 68, 85, 98, 109, 119, 127, 136, 139, 140, 140, 141, 141, 141, 141, 141, 141},
        {0, 0, 0, 0, 0, 0, 0, 26, 39, 49, 59, 77, 90, 101, 111, 120, 128, 134, 136, 136, 137, 137, 137, 137, 137, 137},
        {0, 0, 0, 0, 0, 0, 0, 0, 25, 38, 49, 68, 83, 94, 104, 113, 121, 129, 132, 132, 133, 133, 133, 133, 133, 133},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 38, 60, 75, 87, 97, 106, 115, 121, 127, 129, 129, 129, 129, 129, 129, 129},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 51, 67, 80, 90, 99, 107, 115, 122, 125, 126, 126, 126, 126, 126, 126}};

    ubyte2 column = (ubyte2) (noLoadVoltage - VOLTAGE_MIN) / VOLTAGE_STEP;
    ubyte2 row    = (ubyte2) (rpm - RPM_MIN) / RPM_STEP;
    ubyte1 value = POWER_LIM_LUT_80[row][column];
    return value;
}
