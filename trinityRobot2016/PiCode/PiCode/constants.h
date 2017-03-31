constexpr double PI = 3.1415926535;
typedef enum visionTarget { NONE, SAFEZONETARGET, CRADLEREDTARGET, CRADLEGREENTARGET, CRADLEBLUETARGET } VisionTarget;
constexpr int FLAMESENSORTHRESHOLD = 0;
//if gridValue is below or equal to .25, treat it as clear
constexpr double CLEARTHRESHOLD = .25;
//we would have had to have had this have to have had code

//gridValue constants
constexpr int UNKNOWN = -1;
constexpr int CLEAR = 0;
constexpr int WALL = 1;
constexpr int FLAME = 2;
constexpr int CANDLE = 3;
constexpr int EXTINGUISHED = 4;
constexpr int SAFEZONE = 5;
constexpr int CRADLERED = 6;
constexpr int CRADLEGREEN = 7;
constexpr int CRADLEBLUE = 8;

//size constants
constexpr int ARENALENGTH_CM = 244;
constexpr int CELLSIZE_CM = 1;
constexpr int ROBOT_DIAMETER_CM = 31;
//big enough to hold entire maze no matter where we start
#define GRIDSIZE_CELLS 5 * ARENALENGTH_CM / CELLSIZE_CM
