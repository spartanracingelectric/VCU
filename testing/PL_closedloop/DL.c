
#include "powerLimit.h"
#include "mathFunctions.h"
#define VOLTAGE_STEP     (int) 5        //float voltageStep = (Voltage_MAX - Voltage_MIN) / (NUM_V - 1);
#define RPM_STEP         (int) 160      //int rpmStep = (RPM_MAX - RPM_MIN) / (NUM_S - 1); 

void PL_populateHashTable(HashTable* table)

{
    /*
    voltage is x axis
    rpm is y axis 
    values are torque in Nm
    */
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------
    // 80 KWH LIMIT <------------------------------------------------------------    
    const int lookupTable[26][26] = {
            {2309, 2309, 1988, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {2216, 2294, 1805, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {2053, 2141, 1613, 2280, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {1870, 1977, 1463, 2140, 2214, 2270, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {1723, 1804, 1303, 1982, 2052, 2140, 2213, 2257, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {1573, 1658, 1165, 1821, 1892, 1984, 2052, 2127, 2184, 2256, 2308, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {1437, 1521, 1031, 1683, 1753, 1826, 1896, 1992, 2052, 2127, 2184, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {1308, 1377, 898, 1536, 1611, 1699, 1772, 1843, 1928, 1994, 2065, 2215, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309, 2309},
            {1183, 1259, 765, 1415, 1500, 1573, 1642, 1715, 1786, 1855, 1921, 2084, 2215, 2264, 2265, 2268, 2269, 2271, 2269, 2272, 2271, 2273, 2273, 2273, 2273, 2273},
            {1064, 1140, 630, 1308, 1383, 1454, 1531, 1594, 1667, 1730, 1805, 1977, 2087, 2159, 2163, 2166, 2168, 2169, 2168, 2165, 2171, 2172, 2172, 2172, 2172, 2172},
            {946, 1036, 484, 1195, 1273, 1331, 1403, 1476, 1553, 1621, 1689, 1846, 1993, 2053, 2069, 2069, 2074, 2076, 2077, 2078, 2077, 2079, 2081, 2081, 2077, 2077},
            {839, 927, 316, 1087, 1164, 1229, 1308, 1375, 1440, 1509, 1574, 1738, 1857, 1965, 1981, 1986, 1988, 1989, 1990, 1992, 1994, 1994, 1994, 1995, 1994, 1993},
            {727, 813, 1, 976, 1057, 1132, 1205, 1274, 1342, 1395, 1467, 1637, 1762, 1857, 1904, 1907, 1909, 1910, 1912, 1913, 1914, 1915, 1915, 1915, 1916, 1916},
            {610, 707, 1, 880, 953, 1032, 1103, 1172, 1244, 1308, 1378, 1534, 1660, 1772, 1818, 1833, 1835, 1837, 1840, 1839, 1840, 1842, 1841, 1842, 1842, 1843},
            {484, 594, 1, 773, 858, 936, 1007, 1077, 1147, 1203, 1277, 1438, 1566, 1659, 1745, 1766, 1767, 1768, 1770, 1771, 1774, 1773, 1774, 1775, 1775, 1774},
            {339, 472, 1, 673, 757, 837, 911, 987, 1049, 1115, 1194, 1347, 1475, 1574, 1697, 1700, 1705, 1706, 1707, 1707, 1709, 1709, 1711, 1711, 1711, 1711},
            {119, 326, 1, 564, 657, 742, 816, 893, 962, 1031, 1102, 1257, 1384, 1497, 1586, 1635, 1644, 1646, 1647, 1648, 1650, 1650, 1651, 1652, 1653, 1652},
            {1, 102, 1, 445, 550, 641, 724, 801, 876, 942, 1019, 1168, 1294, 1395, 1501, 1565, 1586, 1591, 1592, 1595, 1594, 1595, 1596, 1597, 1597, 1598},
            {1, 1, 1, 302, 432, 537, 628, 708, 783, 856, 931, 1091, 1205, 1321, 1407, 1509, 1531, 1539, 1541, 1542, 1543, 1544, 1544, 1545, 1546, 1546},
            {1, 1, 1, 61, 290, 420, 524, 614, 693, 767, 847, 1007, 1133, 1245, 1340, 1418, 1477, 1491, 1492, 1495, 1494, 1495, 1496, 1496, 1497, 1498},
            {1, 1, 1, 1, 36, 279, 409, 511, 599, 679, 762, 928, 1054, 1165, 1263, 1348, 1422, 1439, 1446, 1448, 1448, 1449, 1450, 1451, 1452, 1416},
            {1, 1, 1, 1, 1, 1, 270, 397, 500, 588, 675, 847, 979, 1091, 1187, 1267, 1361, 1392, 1403, 1403, 1405, 1406, 1407, 1408, 1408, 1411},
            {1, 1, 1, 1, 1, 1, 1, 258, 386, 489, 587, 767, 902, 1009, 1113, 1201, 1277, 1344, 1359, 1364, 1365, 1366, 1367, 1368, 1368, 1368},
            {1, 1, 1, 1, 1, 1, 1, 1, 248, 378, 492, 683, 825, 942, 1039, 1131, 1213, 1291, 1317, 1324, 1327, 1328, 1329, 1329, 1329, 1330},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 238, 384, 602, 752, 870, 968, 1058, 1145, 1212, 1273, 1288, 1290, 1292, 1291, 1292, 1293, 1293},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 249, 513, 673, 797, 901, 993, 1074, 1151, 1217, 1247, 1256, 1257, 1258, 1259, 1258, 1259}};
    const int VOLTAGE_MIN = 280;
    const int VOLTAGE_MAX = 405;
    const int RPM_MIN = 2000;
    const int RPM_MAX = 6000;
    const int NUM_V = 26;
    const int NUM_S = 26;
    for(int row = 0; row < NUM_S; ++row) {
        for(int column = 0; column < NUM_V; ++column) {
            int voltage = VOLTAGE_MIN + column * VOLTAGE_STEP;
            int rpm   = RPM_MIN + row * RPM_STEP;
            int value = lookupTable[(int)row][(int)column];
            HashTable_insertPair(table, voltage, rpm, value);
        }
    }
}
int PL_getTorqueFromLUT(HashTable* torqueHashTable, int voltage, int rpm){
    int voltageFloor      = int_lowerStepInterval(voltage,5);
    int voltageCeiling    = int_upperStepInterval(voltage,5);
    int rpmFloor          = int_lowerStepInterval(rpm,160);
    int rpmCeiling        = int_upperStepInterval(rpm,160);
    
    // Calculating these now to speed up interpolation later in method

    // Retrieve torque values from the hash table for the four corners
    int vFloorRFloor      = HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    int vFloorRCeiling    = HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    int vCeilingRFloor    = HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    int vCeilingRCeiling  = HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Early escape in case values are the same. May want to make more complex for scenarios such as 2 of the values are the same.
    if(vFloorRFloor == vFloorRCeiling && vCeilingRFloor == vCeilingRCeiling)
    {
        return vFloorRFloor;
    }

    
    int horizontal_Interp = (((vCeilingRFloor - vFloorRFloor) / 5.0) + ((vCeilingRCeiling - vFloorRCeiling) / 5.0)) / 2.0;
    int vertical_Interp = (((vFloorRCeiling - vFloorRFloor) / 160.0) + ((vCeilingRCeiling - vCeilingRFloor) / 160.0)) / 2.0;
    // Calculate interpolation values
   int gainValueHoriz = voltage % 5;
    int gainValueVertical = rpm % 160;

    // Final TQ from LUT
    int TQ =  (gainValueHoriz * horizontal_Interp) + (gainValueVertical * vertical_Interp) + vFloorRFloor;
    
    /*
    float4 horizontalInterpolation = (((vCeilingRFloor - vFloorRFloor) / VOLTAGE_STEP) + ((vCeilingRCeiling - vFloorRCeiling) / VOLTAGE_STEP)) / 2.0;
    float4 verticalInterpolation   = (((vFloorRCeiling - vFloorRFloor) / RPM_STEP) + ((vCeilingRCeiling - vCeilingRFloor) / RPM_STEP)) / 2.0;

    // Calculate gains
    float4 gainValueHorizontal = (float4)fmod(voltage, VOLTAGE_STEP);
    float4 gainValueVertical   = (float4)fmod(rpm, RPM_STEP);

    // Combine interpolated values
    intlutTorque = (gainValueHorizontal * horizontalInterpolation) + (gainValueVertical * verticalInterpolation) + vFloorRFloor;
    */
    return TQ;  // Adjust gain if necessary

}  // Find the floor and ceiling values for voltage and rpm)

int PL_getTorqueFromLUT2(HashTable* torqueHashTable, int voltage, int rpm){    // Find the floor and ceiling values for voltage and rpm
    
    // LUT Lower Bounds
    int VOLTAGE_MIN      = 280;
    int RPM_MIN          = 2000;

    // Calculating hashtable keys
    int rpmInput         = rpm - RPM_MIN;
    int voltageInput     = voltage - VOLTAGE_MIN;
    int voltageFloor     = int_lowerStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    int voltageCeiling   = int_upperStepInterval(voltageInput, VOLTAGE_STEP) + VOLTAGE_MIN;
    int rpmFloor         = int_lowerStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    int rpmCeiling       = int_upperStepInterval(rpmInput, RPM_STEP) + RPM_MIN;
    
    // Calculating these now to speed up interpolation later in method
    int voltageLowerDiff = voltage - voltageFloor;
    int voltageUpperDiff = voltageCeiling - voltage;
    int rpmLowerDiff     = rpm - rpmFloor;
    int rpmUpperDiff     = rpmCeiling - rpm;

    // Retrieve torque values from the hash table for the four corners
    int vFloorRFloor     = HashTable_getValue(torqueHashTable, voltageFloor, rpmFloor);
    int vFloorRCeiling   = HashTable_getValue(torqueHashTable, voltageFloor, rpmCeiling);
    int vCeilingRFloor   = HashTable_getValue(torqueHashTable, voltageCeiling, rpmFloor);
    int vCeilingRCeiling = HashTable_getValue(torqueHashTable, voltageCeiling, rpmCeiling);

    // Calculate interpolation values
    int stepDivider          = VOLTAGE_STEP      * RPM_STEP;
    long torqueFloorFloor     = vFloorRFloor      * voltageUpperDiff * rpmUpperDiff;
    long torqueFloorCeiling   = vFloorRCeiling    * voltageUpperDiff * rpmLowerDiff;
    long torqueCeilingFloor   = vCeilingRFloor    * voltageLowerDiff * rpmUpperDiff;
    long torqueCeilingCeiling = vCeilingRCeiling  * voltageLowerDiff * rpmLowerDiff;

    // Final TQ from LUT
    return (torqueFloorFloor + torqueFloorCeiling + torqueCeilingFloor + torqueCeilingCeiling) / stepDivider;
}
