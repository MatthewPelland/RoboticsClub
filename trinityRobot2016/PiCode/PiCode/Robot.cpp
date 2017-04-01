#include "Robot.h"
#include <string>
#include <chrono>
#include <iostream>

typedef std::chrono::high_resolution_clock Clock;

Robot::Robot() {
	arduinoSerial = serialOpen("/dev/ttyACM0", 9600);
	level = 1;
	angle = 0;
	angVel = 0;
	sensorDist_cm = 200; //maximum readable distance
	moves = std::vector<Point>();
	unextinguishedCandleCount = 0;
	inRoom = 1;
	std::cout << "before sonar" << std::flush;
	sonic0 = SonicSensor(SONIC1_TRIG, SONIC1_ECHO);//front
        sonic1 = SonicSensor(SONIC2_TRIG, SONIC2_ECHO);//right
        sonic2 = SonicSensor(SONIC3_TRIG, SONIC3_ECHO);//rear
        sonic3 = SonicSensor(SONIC4_TRIG, SONIC4_ECHO);//left
        flameSensor = IRSensor(FLAMESENSORPIN);
        sensor = MotionSensor(wiringPiI2CSetup(0x68));


	startTime = Clock::now();
	currTime = Clock::now();
	prevTime = Clock::now();
	safeZoneLocation = Point(-1, -1);
	currentPosCells = Point(GRIDSIZE_CELLS / 2, GRIDSIZE_CELLS / 2);
	std::cout << arduinoSerial << "\n" << std::flush;
	if (level == 3) {
		babySaved = false;
		cradleFound = false;
		babyObtained = false;
		safeZoneFound = false;
	}
	else {
		//treat lower levels as if level 3 with baby already saved
		babySaved = true;
		cradleFound = true;
		babyObtained = true;
		safeZoneFound = true;
	}
	grid = (gridVal**) malloc(sizeof(gridVal*) * GRIDSIZE_CELLS);
	for (int i = 0; i < GRIDSIZE_CELLS; i++){
		grid[i] = (gridVal*) malloc(sizeof(gridVal) * GRIDSIZE_CELLS);
		for (int j = 0; j < GRIDSIZE_CELLS; j++)
			grid[i][j] = gridVal();
	}
	distanceField = (int**) malloc(sizeof(int*) * GRIDSIZE_CELLS);
  	for (int i = 0; i < GRIDSIZE_CELLS; i++){
		distanceField[i] = (int*)malloc(sizeof(int) * GRIDSIZE_CELLS);
		memset(distanceField[i], -1, sizeof(int) * GRIDSIZE_CELLS);
	}
	while(!serialDataAvail(arduinoSerial));
	serialFlush(arduinoSerial);
        initialScan();//align robot
//	serialPrintf(arduinoSerial, "z %d %d %d\n", 0, currentPosCells.x, currentPosCells.y);
	//arduinoCommands << "z " << 0 << " " << currentPosCells.x << " " << currentPosCells.y << "\n";
}

void Robot::testGyro(){
	updateTime();
	for(int i = 0; i < 10000; i ++){
		updateTime();
		updateAngle(std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
		if(i % 100 == 0){
			std::cout << angle << "\n" << std::flush;
		}
	}
}

//Robot::~Robot() {
//	arduinoCommands.close();
//	for (int i = 0; i < GRIDSIZE_CELLS; i++) {
//		free(grid[i]);
//		free(distanceField[i]);
//	}
//}

//returns boolean on whether or not maze is completed
bool Robot::update() {
	moves.clear();
	scanSurroundings();
	int mazeComplete;
	if(!babySaved)
		mazeComplete = createTargetPath(findNextTarget(true));//ignore candles while baby is not saved
	else
		mazeComplete = createTargetPath(findNextTarget());
	if (unextinguishedCandleCount == 0 && mazeComplete) {//no candles left and maze fully mapped
		routeToStart();
		moveTo(moves);
		return true;
	}
	moveTo(moves);
	return false;
	//moves.clear();
	//if (babySaved) {//this seems legit
	//	//doNormalStuff
	//	scanSurroundings();
	//	int mazeComplete = createTargetPath(findNextTarget());
	//	if (unextinguishedCandleCount == 0 && mazeComplete) {//no unextinguished candles and maze fully mapped
	//			returnToStart();
	//	}
	//}
	//else {//playing level 3 and baby not yet saved
	//	scanSurroundings(true);//ignore candles
	//	if (babyObtained) {
	//		if (safeZoneFound) {
	//			goToSafeZone();
	//			pushBabyOutWindow();
	//			moves.clear();
	//		}
	//		else {
	//			createTargetPath(findNextTarget(true)); //ignore candles
	//		}
	//	}
	//	else if (cradleFound) {
	//		retrieveBaby();
	//	}
	//	else {//cradle, not found, proceed finding unknowns, ignore candles
	//		createTargetPath(findNextTarget(true));
	//	}
	//}
	//moveTo(moves);
}

//prevTime and currTime and std::chrono::time_points recorded in microseconds
void Robot::updateTime() {
	prevTime = currTime;
	currTime = Clock::now();
}

//orients the robot square to the walls of the maze
void Robot::initialScan() {
    double angleDelta = 0;
	double sonarData[360];
	for(int i = 0; i < 360; i ++)
		sonarData[i] = -1;
    serialPrintf(arduinoSerial, "r 180\n");
	//arduinoCommands << "r 120\n";
	updateTime();
	std::cout << "rotating" << std::flush;
	delay(100);
    while (!serialDataAvail(arduinoSerial)) {
		std::cout << "LOOPING" << std::endl;
		updateTime();
		angleDelta += updateAngle((double)std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
		sonarData[(int)(angleDelta + 0 * 90)%360] = sonic0.getDistance()+15;
		sonarData[(int)(angleDelta + 1 * 90)%360] = sonic1.getDistance()+15;
		//sonarData[(int)(angleDelta / (PI / 2.0) * 90) + 2 * 90] = sonic2.getDistance()+7;
		sonarData[(int)(angleDelta + 3 * 90)%360] = sonic3.getDistance()+15;
		delay(5);
    }
	std::cout<< "all done" << std::flush;
	serialFlush(arduinoSerial);
    int mostPerpindicular = 0;
    int bestPerpindicularity = 10000000;
    int perpindicularity;
    for (int i = 0; i < 360; i++) {
		perpindicularity = 0;
		for (int j = -5; j <= 5; j++) {
			perpindicularity += abs(sonarData[(i + j + 360) % 360] - \
						sonarData[i]);
		}
		if (perpindicularity < bestPerpindicularity) {
			bestPerpindicularity = perpindicularity;
			mostPerpindicular = i;
		}
    }
    serialPrintf(arduinoSerial, "r %d\n", mostPerpindicular%90);
	//arduinoCommands << "r " << (mostPerpindicular - 120 < 180 ? mostPerpindicular - 120 : mostPerpindicular - 120 - 360) << "\n";
	angle = 0;
	std::cout << "best" << mostPerpindicular << std::endl;
}

void cleanSonarData(double sonarData[4][360]){
	for(int sensor = 0; sensor < 4; sensor ++){
		for(int i = 0; i < 360; i ++){
			if(abs(sonarData[sensor][i] - sonarData[sensor][(i+359)%360]) > 5 && abs(sonarData[sensor][i] - sonarData[sensor][(i+1)%360]) > 5)
				sonarData[sensor][i] = sonarData[sensor][(i+1)%360];
		}
	}
}

//needs lots of edits once camera shit is figured out
//refactor the shit out of this
void Robot::scanSurroundings(bool ignoreCandles) {//double check that this one is right
	/////////////////////
	//   Gather Data   //
	/////////////////////
	Point candle = Point(-1, -1);//lit Candle
	std::vector<Point> cradles = std::vector<Point>();
	std::vector<Point> windows = std::vector<Point>();
	std::vector<Point> unlitCandles = std::vector<Point>();
	double sonarData[4][360];
	for (int i = 0; i < 4; i++) {
		for(int j = 0; j < 360; j ++)
			sonarData[i][j] = -1;
	}
	int flameSensorData[360];
	memset(flameSensorData, -1, sizeof(int) * 360);
	//void cameraData[360];
	serialPrintf(arduinoSerial, "r 360\n");
	//arduinoCommands << "r 360\n";
	updateTime();
	std::cout << "gonna scan Surroundings" << std::endl;
	serialFlush(arduinoSerial);
	while (!serialDataAvail(arduinoSerial)) {
		updateTime();
		updateAngle(std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
		//flameSensorData[(int)scanAngle] = flameSensor.getFireIntensity();
			sonarData[0][(int)(angle + 0 * 90 + 90) % 360] = sonic0.getDistance() + 15;
			sonarData[1][(int)(angle + 1 * 90 + 90) % 360] = sonic1.getDistance() + 15;
			//sonarData[2][(int)(angle + 2 * 90 + 90) % 360] = sonic2.getDistance() + 7; //offset in by 8 cm
			sonarData[3][(int)(angle + 3 * 90 + 90) % 360] = sonic3.getDistance() + 15;
	}
	std::cout << "finishedScanning" << std::endl;
	serialFlush(arduinoSerial);

	//////////////////////
	//   Analyze Data   //
	//////////////////////

	//also clears out false window data, so necessary for all levels
	windows = checkForWindow(sonarData); 
	if (level == 3) {
		unlitCandles = locateCandles(sonarData);
		if(!babyObtained)
			cradles = checkForCradle(sonarData);
	}



	cleanSonarData(sonarData);
	for (int sensor = 0; sensor < 4; sensor++) {
		for (int i = 0; i < 360; i++) {
			//do sonar stuff
			double distanceReading = sonarData[sensor][i];
			if (distanceReading < sensorDist_cm && distanceReading > 0) {
				int targetCellX = int((distanceReading * cos((i + 90 * sensor + 90)*PI/180)) / CELLSIZE_CM + currentPosCells.x);
				int targetCellY = int((distanceReading * sin((i + 90 * sensor + 90)*PI/180)) / CELLSIZE_CM + currentPosCells.y);
				gridVal* targetCell = &(grid[targetCellX][targetCellY]);
				//check fire sensor
				if (sensor == 0) {//front sensor
					if (flameSensorData[i] >= FLAMESENSORTHRESHOLD) {
						targetCell->cellType = FLAME;
						candle = Point(targetCellX, targetCellY);
					}
				}

				if (targetCell->cellType <= 1) {//in the wall/clear gradient or -1 (unknown)
					updateGridVal(targetCellX, targetCellY, WALL);
				}
				//increase apparent size of walls to compensate for the size of the robot 
				//so we don't have to keep track of distance to walls elsewhere.
				//	for (int i = -ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; i <= ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; i++) {
				//		for (int j = -ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; j <= ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; j++) {
				//			gridVal* offsetTarget = &(grid[targetCellX + i][targetCellY + j]);
				//			if (offsetTarget->cellType <= 1) {
				//				updateGridVal(targetCellX + i, targetCellY + j, WALL);
				//			}
				//		}
				//	}

			}
			//every cell between robot and wall is clear
			//if(distanceReading < 500){
				for (int dist = 0; dist < distanceReading-1; dist++) {
					int cellX = int(dist * cos((i + 90 + sensor * 90) * PI / 180) / CELLSIZE_CM + currentPosCells.x);
					int cellY = int(dist * sin((i + 90 + sensor * 90) * PI / 180) / CELLSIZE_CM + currentPosCells.y);
					if (cellX >= 0 && cellX < GRIDSIZE_CELLS && cellY >= 0 && cellY < GRIDSIZE_CELLS && grid[cellX][cellY].cellType <= 1)
						updateGridVal(cellX, cellY, CLEAR);//updateGridVal(cellX, cellY, CLEAR);
				}
			//}
		}
	}

	for (int sensor = 0; sensor < 4; sensor++) {
		for (int i = 0; i < 360; i++) {
			//do sonar stuff
			double distanceReading = sonarData[sensor][i];
			if (distanceReading < sensorDist_cm && distanceReading > 0) {
				int targetCellX = int((distanceReading * cos(i + 90 * sensor)) / CELLSIZE_CM + currentPosCells.x);
				int targetCellY = int((distanceReading * sin(i + 90 * sensor)) / CELLSIZE_CM + currentPosCells.y);
				gridVal* targetCell = &(grid[targetCellX][targetCellY]);
				if(findOpenNeighbors(Point(targetCellX, targetCellY)).size() > 2)
				updateGridVal(targetCellX, targetCellY, CLEAR);// -> cellType = 0;
				else{
					for (int i = -2; i <= 2; i++) {
						for (int j = -2 ; j <= 2; j++) {
							gridVal* offsetTarget = &(grid[targetCellX + i][targetCellY + j]);
							if (offsetTarget->cellType <= 1) {
							//	updateGridVal(targetCellX + i, targetCellY + j, WALL);
							}
						}
					}
				}
			}
		}
	}


	std::cout << "hello" << std::endl;
/*	// 1. lit candle
	if (candle.x != -1) {
		if (!ignoreCandles)
			extinguishCandle(candle);//immediately go put out the candle
		else
			unextinguishedCandleCount++;
	}

	//2. unlitCandles
	if (unlitCandles.size() > 0) {
		for (unsigned int i = 0; i < unlitCandles.size(); i++) {
			if (grid[unlitCandles[i].x][unlitCandles[i].y].cellType != FLAME  
				&& grid[unlitCandles[i].x][unlitCandles[i].y].cellType != CANDLE 
				&& grid[unlitCandles[i].x][unlitCandles[i].y].cellType != EXTINGUISHED) {
				grid[unlitCandles[i].x][unlitCandles[i].y].cellType = CANDLE;
				unextinguishedCandleCount++;
			}
		}
	}

	//3. window
	if (!babySaved && !safeZoneFound && windows.size() > 0) {
		for (unsigned int i = 0; i < windows.size(); i++) {
			Point direction;
			double turnAngle;
			for (double j = 0; j < 2 * PI; j += PI / 2) {
				if (grid[(int)(windows[i].x + cos(j) * ROBOT_DIAMETER_CM / 2 + 1)][(int)(windows[i].y + sin(j) * ROBOT_DIAMETER_CM / 2 + 1)].cellType == CLEAR);
				direction = Point((int)cos(j), (int)sin(j));
				turnAngle = (PI / 2 + j) * 180 / PI;;
			}
			Point target = Point(windows[i].x + direction.x * ROBOT_DIAMETER_CM / 2 + 1, windows[i].y + direction.y * ROBOT_DIAMETER_CM / 2 + 1);
			createTargetPath(target);
			moveTo(moves, true);
			//determine which way to rotate wait that information is just straight from direction, fucking idiot.
			serialPrintf(arduinoSerial, "r %d\n", (int)turnAngle);
			//arduinoCommands << "r " << "turnAngle" << "\n";
			waitForDoneConfirmation();
			bool isSafeZone = checkImageForSafeZone();
			if (isSafeZone){
				safeZoneFound = true;
				safeZoneLocation = windows[i];
				if (babyObtained) {
					serialPrintf(arduinoSerial, "w 10\n");
					//arduinoCommands << "w 10\n";
					waitForDoneConfirmation();
					babySaved = true;
				}
				serialPrintf(arduinoSerial, "r %d\n", -(int)turnAngle);
				//arduinoCommands << "r " << -turnAngle << "\n";
				break;
			}
			else {
				serialPrintf(arduinoSerial, "r %d\n", -(int)turnAngle);
				//arduinoCommands << "r " << -turnAngle << "\n";
				waitForDoneConfirmation();
			}
		}
	}

	//4. cradle
	if (!babyObtained && cradles.size() != 0) {
		//DO LOTS OF SHIT IN HERE LIKE GO SAVE THE BABY
		//make this a function dummkopf
		int moveDistance = 5;//cm after done getting baby
		for (unsigned int i = 0; i < cradles.size(); i++) {
			Point direction;
			double turnAngle;
			for (double j = 0; j < 2 * PI; j += PI / 2) {
				if (grid[(int)(cradles[i].x + cos(j) * ROBOT_DIAMETER_CM / 2 + 1)][(int)(cradles[i].y + sin(j) * ROBOT_DIAMETER_CM / 2 + 1)].cellType == CLEAR);
				direction = Point((int)cos(j), (int)sin(j));
				turnAngle = (PI / 2 + j) * 180 / PI;
			}
			Point target = Point(cradles[i].x + direction.x * ROBOT_DIAMETER_CM / 2 + 1, cradles[i].y + direction.y * ROBOT_DIAMETER_CM / 2 + 1);
			createTargetPath(target);
			moveTo(moves, true);
			//determine which way to rotate wait that information is just straight from direction, fucking idiot.
			serialPrintf(arduinoSerial, "r %d\n", (int)turnAngle);
			//arduinoCommands << "r " << turnAngle << "\n";
			waitForDoneConfirmation();
			bool isCradle = checkImageForCradle();
			if (isCradle) {
				digitalWrite(VISIONLEDPIN, HIGH);
				serialPrintf(arduinoSerial, "c %d\n", moveDistance);
				//arduinoCommands << "c " << moveDistance << "\n";
				waitForDoneConfirmation();
				serialPrintf(arduinoSerial, "r %d\n", -(int)turnAngle);
				//arduinoCommands << "r " << -turnAngle << "\n";
				waitForDoneConfirmation();
				serialPrintf(arduinoSerial, "z 0 %d %d\n", safeZoneLocation.x + direction.x * moveDistance, safeZoneLocation.y + direction.y * moveDistance);
				currentPosCells.x = safeZoneLocation.x + direction.x * moveDistance;
				currentPosCells.y = safeZoneLocation.y + direction.y * moveDistance;
				//arduinoCommands << "z " << safeZoneLocation.x + direction.x * moveDistance << safeZoneLocation.y + direction.y * moveDistance << "\n";
				babyObtained = true;
				digitalWrite(VISIONLEDPIN, LOW);
				if (safeZoneFound) {
					goSaveBaby(); //does everything
					babySaved = true;
				}
				break;
			}
			else {
				serialPrintf(arduinoSerial, "r %d\n", -(int)turnAngle);
				//arduinoCommands << "r " << turnAngle << "\n";
				waitForDoneConfirmation();
			}
		}
	}*/
}

void Robot::waitForDoneConfirmation() {
	while (!serialDataAvail(arduinoSerial));
	serialFlush(arduinoSerial);
}

bool Robot::checkImageForCradle(){
	return false;
}

bool Robot::checkImageForSafeZone(){
	return false;
}

//good
void Robot::updateGridVal(int cellX, int cellY, int type) {
	if(cellX > GRIDSIZE_CELLS || cellX < 0 || cellY < 0 || cellY > GRIDSIZE_CELLS)
		return;
	grid[cellX][cellY].cellType = (grid[cellX][cellY].cellType*grid[cellX][cellY].timesScanned + type) / (grid[cellX][cellY].timesScanned++ + 1);
}

//check later
int Robot::createTargetPath(Point target, int thresholdDistance) {
    //fills moves with list of all moves necessary to reach target tile
    //moves contains waypoints at each point the robot switches direction

	//if there are no more unknowns on the map
	if (target.x == -1)
		return 1; //map completed
				  //otherwise...
	computeDistanceField(target);
	int currentDist = distanceField[target.x][target.y];
	Point currentSquare = target;
	Point prevDirection(0, 0);
	Point currDirection(0, 0);
	for (int reverseCount = 0; reverseCount < distanceField[target.x][target.y]; reverseCount++) {
		std::vector<Point> openNeighbors;
		openNeighbors = findOpenNeighbors(currentSquare);
		for (unsigned int i = 0; i < openNeighbors.size(); i++) {
			//if the neighbor is one square closer to robot than the current square
			if (distanceField[openNeighbors[i].x][openNeighbors[i].y] == currentDist - 1) {
				currentDist = distanceField[openNeighbors[i].x][openNeighbors[i].y];
				prevDirection.x = currDirection.x;
				prevDirection.y = currDirection.y;
				currDirection.x = openNeighbors[i].x - currentSquare.x;
				currDirection.y = openNeighbors[i].y - currentSquare.y;

				if (prevDirection.x != currDirection.x || prevDirection.y != currDirection.y //there was a change in direction
					|| (moves.size() == 0 && (abs(currentSquare.x - target.x) + abs(currentSquare.y - target.y)) > thresholdDistance) //we reached a threshold distance from the endPoint this shouldn't be encessary, double check
					|| distance(currentSquare, moves[0]) > 40){ //we have a current distance travel over 40cm.  So our movement can't drift us too much
					moves.insert(moves.begin(), currentSquare);
				}
				currentSquare = openNeighbors[i];
			}
		}
	}
    
	return 0;//map not completed
}

//can be absorbed into computeDistanceField but don't screw with it.
Point Robot::findNextTarget(bool ignoreCandles) {//This one's pretty fast
	for (int i = 0; i < GRIDSIZE_CELLS; i++) {
		memset(distanceField[i], -1, sizeof(int) * GRIDSIZE_CELLS);
	}
	//finds the closest unknown tile
	std::vector<Point> boundary;
	boundary.push_back(Point(currentPosCells.x, currentPosCells.y));
	distanceField[currentPosCells.x][currentPosCells.y] = 0;
	while (boundary.size() > 0) {
		Point checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<Point> openNeighbors;
		openNeighbors = findOpenNeighbors(checking);
		for (unsigned int i = 0; i < openNeighbors.size(); i++) {
			gridVal neighbor = grid[openNeighbors[i].x][openNeighbors[i].y];
			if ((neighbor.cellType == UNKNOWN) || ((neighbor.cellType == FLAME || neighbor.cellType == CANDLE) && !ignoreCandles)) {
				distanceField[openNeighbors[i].x][openNeighbors[i].y] = distanceField[checking.x][checking.y] + 1;
				return openNeighbors[i]; //closest unknown/candle tile as ordered
			}
			else if (distanceField[openNeighbors[i].x][openNeighbors[i].y] == -1) {
					boundary.push_back(openNeighbors[i]);
					distanceField[openNeighbors[i].x][openNeighbors[i].y] = distanceField[checking.x][checking.y] + 1;
				}
		}
    }
    return Point(-1, -1);  //no unknown tiles left, maze completely mapped
}

void Robot::extinguishCandle(Point target) {
	if (!inRoom) {
		createTargetPath(closestOpenCell(target), 30);
		moveTo(moves);
	}
	digitalWrite(FLAMELEDPIN, HIGH);
	//actually extinguish the candle
	double angleToMove = customAtan(target.y - currentPosCells.y, target.x - currentPosCells.x);
	serialPrintf(arduinoSerial, "r %d\n", (int)(angleToMove * 180 / PI - 90));
	//arduinoCommands << "r " << (int)(angleToMove * 180 / PI - 90) << "\n";
	while (flameSensor.getFireIntensity() > FLAMESENSORTHRESHOLD) {
		serialPrintf(arduinoSerial, "e\n");//for extinguish
		//arduinoCommands << "e\n";
		waitForDoneConfirmation();
	}
	digitalWrite(FLAMELEDPIN, LOW);
	serialPrintf(arduinoSerial, "r %d\n", (int)(-angleToMove * 180 / PI));
	//arduinoCommands << "r " << (int)(-angleToMove * 180 / PI) << "\n";
	//mark all candles in the area as extinguished
	for(int i = -2; i <= 2; i ++)
		for(int j = -2; j <= 2; j++)
			if(grid[target.x + i][target.y + j].cellType == CANDLE || grid[target.x + i][target.y + j].cellType == FLAME)
				grid[target.x + i][target.y + j].cellType = EXTINGUISHED;
	waitForDoneConfirmation();
}

//good
Point Robot::closestOpenCell(Point target) {//for getting to candles
	//I could also use this for getting into position for the cradle and windows
	//cell robot can fit in not first open Cell
	std::vector<Point> boundary;
	boundary.push_back(Point(target.x, target.y));
	while (boundary.size() > 0) {
		Point checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<Point> openNeighbors;
		openNeighbors = findOpenNeighbors(checking);
		for (unsigned int i = 0; i < openNeighbors.size(); i++) {
			Point current = openNeighbors[i];
			if (grid[current.x][current.y].cellType <= CLEARTHRESHOLD && distance(current, target) > ROBOT_DIAMETER_CM / 2 + 2) {
				return current;
			}
			else {
				if (distance(target, current) > distance(target, checking)) {
					boundary.push_back(current);
				}
			}
		}
	}
	return Point(-1, -1);
}

//good
double Robot::distance(Point a, Point b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

//prolly good
void Robot::computeDistanceField(Point target) {
	for (int i = 0; i < GRIDSIZE_CELLS; i++) {
		memset(distanceField[i], -1, sizeof(int) * GRIDSIZE_CELLS);
	}
	//finds the closest unknown tile
	std::vector<Point> boundary;
	boundary.push_back(Point(currentPosCells.x, currentPosCells.y));
	distanceField[currentPosCells.x][currentPosCells.y] = 0;
	while (boundary.size() > 0) {
		Point checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<Point> openNeighbors;
		openNeighbors = findOpenNeighbors(checking);
		for (unsigned int i = 0; i < openNeighbors.size(); i++) {
			if (openNeighbors[i].x == target.x && openNeighbors[i].y == target.y) {
				distanceField[openNeighbors[i].x][openNeighbors[i].y] = distanceField[checking.x][checking.y] + 1;
			}
			else {
				if (distanceField[openNeighbors[i].x][openNeighbors[i].y] == -1) {
					boundary.push_back(openNeighbors[i]);
					distanceField[openNeighbors[i].x][openNeighbors[i].y] = distanceField[checking.x][checking.y] + 1;
				}
			}
		}
	}
}

//good
std::vector<Point> Robot::findOpenNeighbors(Point currentPos) {
    std::vector< Point > openNeighbors;
    for (int x_offset = -1; x_offset < 2; x_offset++) {
		for (int y_offset = -1; y_offset < 2; y_offset++) {
			if (!is_diagonal_candidate(x_offset, y_offset) && \
			grid[currentPos.x + x_offset][currentPos.y + y_offset].cellType <= CLEARTHRESHOLD &&  grid[currentPos.x + x_offset][currentPos.y + y_offset].cellType >= 0) {
			openNeighbors.push_back(Point(currentPos.x + x_offset,
							  currentPos.y + y_offset));
			}
		}
    }
    return openNeighbors;
}

//good

//x\y 1  0 -1
//1   n     n
//0      X
//-1  n     n

bool Robot::is_diagonal_candidate(int x_offset, int y_offset) {
	return ((x_offset + y_offset + 2) % 2 == 0);
}

void Robot::moveTo(std::vector<Point> moves, bool takePictures){
    // send commands to Arduino to go somewhere by grid coordinates
	updateTime();
	if (moves.size() >= 0) {
		for (unsigned int i = 0; i < moves.size(); i++) {
			serialPrintf(arduinoSerial, "m %d %d\n", moves[i].x, moves[i].y);
			//arduinoCommands << "m " << moves[i].x << " " << moves[i].y << "\n";
			while (!serialDataAvail(arduinoSerial)) {
				updateTime();
				updateAngle(std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
			}
			if (takePictures)
				takePicture();
			char receive[15];
			int j = 0;
			while ((receive[j] = serialGetchar(arduinoSerial)) != '\n')
				j++;
			sscanf(receive, "%d", &(currentPosCells.x));
			sscanf(receive, " %d", &(currentPosCells.y));
			serialPrintf(arduinoSerial, "r %d\n", -(int)angle);
			//arduinoCommands << "r " << -angle << "\n";
			angle = 0;
			int xCorrect = 0;
			int yCorrect = 0;

			//compare alleged sonar values to real sonar values to get real pos
			int allegedSonar0 = distanceToWall(0, 1);
			int allegedSonar1 = distanceToWall(1, 0);
			int allegedSonar2 = distanceToWall(0, -1);
			int allegedSonar3 = distanceToWall(-1, 0);
			double realSonar0 = sonic0.getDistance() + 15;
			double realSonar1 = sonic1.getDistance() + 15;
			double realSonar2 = -1;//sonic2.getDistance() + 7;
			double realSonar3 = sonic3.getDistance() + 15;

			if (allegedSonar0 != -1 && abs(allegedSonar0 - realSonar0) < 10) {//front
				yCorrect = allegedSonar0 - realSonar0;
			}
			if (allegedSonar2 != -1 && abs(allegedSonar2 - realSonar2) < 10) {//rear
				yCorrect = -(allegedSonar2 - realSonar2);
			}
			if (allegedSonar1 != -1 && abs(allegedSonar1 - realSonar1) < 10) {//right
				xCorrect = allegedSonar1 - realSonar1;
			}
			if (allegedSonar3 != -1 && abs(allegedSonar3 - realSonar3) < 10) {//left
				xCorrect = -(allegedSonar3 - realSonar3);
			}
			serialPrintf(arduinoSerial, "m %d %d\n", xCorrect, yCorrect);
			//arduinoCommands << "m " << xCorrect << " " << yCorrect << "\n";
			j = 0;
			while ((receive[j] = serialGetchar(arduinoSerial)) != '\n')
				j++;
			serialPrintf(arduinoSerial, "z 0 %d %d\n", currentPosCells.x, currentPosCells.y);
			//arduinoCommands << "z 0 " << currentPosCells.x << currentPosCells.y << "\n";
		}
	}
}

int Robot::distanceToWall(int xDirect, int yDirect) {
	Point tracking = Point(currentPosCells.x, currentPosCells.y);
	while (grid[tracking.x][tracking.y].cellType == CLEAR) {
		tracking.x += xDirect;
		tracking.y += yDirect;
	}
	if (grid[tracking.x][tracking.y].cellType == UNKNOWN)
		return -1;
	else
		return tracking.x - currentPosCells.x + tracking.y - currentPosCells.y;
}

double Robot::updateAngle(double timeDelta) {
    double angleDelta = sensor.getGyroZ() * timeDelta/1000000;//microseconds
    angle += angleDelta*2;
	return angleDelta*2;
}

std::vector<Point> Robot::checkForCradle(double sonarData[4][360]) {
	double averages[360];
	double length = 0;
	std::vector<Point> midpoints;

	for (int i = 0; i < 360; i++) { //find averages
		averages[i] = (sonarData[0][i] + sonarData[1][i] + sonarData[2][i] + sonarData[3][i]) / 4;
	}

	//find initial points and vector lengths
	DoublePoint lastP1 = DoublePoint(averages[0] * cos(0 * PI / 180), averages[0] * sin(0 * PI / 180));
	DoublePoint lastP2 = DoublePoint(averages[1] * cos(1 * PI / 180), averages[1] * sin(1 * PI / 180));
	double lastLen = sqrt(pow((lastP2.x - lastP1.x), 2) + pow((lastP2.y - lastP1.y), 2)); //length of last vector
	length = lastLen;
	DoublePoint initialPoint = lastP1;
	bool finished = false;

	for (int i = 0; i < 360 || ! finished; i++) { //degree by degree
									//current two points
		finished = false;
		DoublePoint p1 = DoublePoint(averages[(i)%360] * cos(i * PI / 180), averages[(i)%360] * sin(i * PI / 180));
		DoublePoint p2 = DoublePoint(averages[(i + 1)%360] * cos((i + 1) * PI / 180), averages[(i + 1)%360] * sin((i + 1) * PI / 180));

		double currentLen = sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2)); //length of current vector

																				 //angle between last vector and currernt vector
	//	double angle = acos(((lastP2.x - lastP1.x) * (p2.x - p1.x) + (lastP2.y - lastP1.y) * (p2.y - p1.y)) / (lastLen * currentLen));

		if (!(abs(p2.x - initialPoint.x) > 1 && abs(p2.y - initialPoint.y) > 2)) {
			length += currentLen; //if there's no corner add to current length
		}

		else { //corner
			if (length <= 17 && length >= 13) { //between 13 and 17 cm -- cradle length wall has been found
				Point midpoint = Point(((initialPoint.x + p2.x) / 2), ((initialPoint.y + p2.y) / 2)); //midpoint of vector
				midpoints.push_back(midpoint);
			}
			initialPoint = p2; //new initial point for next wall
			length = 0; //new length
			finished = true;
		}

		//reset last variables
		lastP1 = p1;
		lastP2 = p2;
		lastLen = currentLen;
	}
	return midpoints;
}

std::vector<Point> Robot::checkForWindow(double sonarData[4][360]) {
	std::vector<Point> windows;
	double average;
	Point window;
	for (int i = 0; i < 360; i++) {
		average = (sonarData[1][i] + sonarData[2][i] + sonarData[3][i]) / 3;
		if ((sonarData[0][i] - average) > 10){
			//we found a window!!!
			sonarData[0][i] = average;
			//introducing the jankiest for loop I've ever written
			double j;
			for (j = 0; (sonarData[0][i] - average) > 10; i++, j += .5) {
				average = (sonarData[1][i] + sonarData[2][i] + sonarData[3][i]) / 3;
				sonarData[0][i] = average;
			}
			double middleAverage = (sonarData[1][(int)j] + sonarData[2][i] + sonarData[3][(int)j]) / 3;
			windows.push_back(Point(middleAverage*cos((int)j*PI / 180), middleAverage*sin((int)j*PI / 180)));
		}
	}
	return windows;
}

std::vector<Point> Robot::findAberrantMinimums
 (double sonarData[360],
	const double slope_threshold = 20,
	const double aberration_size_threshold = 5) {
	const int max = 360;
	std::vector<double> aberrant_angles;
	std::vector<Point> candleLocations;
	std::vector<int> steep_slope_angles;
	std::vector<double> slopes(max / 2);
	for (int i = 0; i < max - 1; i += 2) {
		slopes[i] = fabs(sonarData[i] - sonarData[i + 1]);
		if (slopes[i] > slope_threshold) {
			steep_slope_angles.push_back(i);
		}
	}
	for (unsigned int i = 0; i < steep_slope_angles.size() - 1; ++i) {
		const int &angle1 = steep_slope_angles[i];
		const int &angle2 = steep_slope_angles[i + 1];
		const int aberration_size = angle2 - angle1;
		if (aberration_size < aberration_size_threshold) {
			double candleAngle = (angle2 - angle1) / 2;
			aberrant_angles.push_back((angle2 + angle1) / 2);
			candleLocations.push_back(Point(currentPosCells.x + cos(candleAngle) * sonarData[(int)candleAngle], currentPosCells.y + sin(candleAngle) * sonarData[(int)candleAngle]));
		}
	}

	return candleLocations;
}


std::vector<Point> Robot::locateCandles(double sonarData[4][360]) {
	double averages[360];
	for (int i = 0; i < 360; i++) {
		averages[i] = (sonarData[1][i] + sonarData[2][i] + sonarData[3][i] + sonarData[4][i]) / 4;
	}
	    // identify a steep dip in sonarData that returns to normal
//
//			 +      distance  from bot
//			 +      | *                               * *
//			 +      |   *                          *      *
//			 +      |      *                    *      **    *
//			 +      |         *              *                  *
//			 +      |                                                          *
//			 +      |             *  .  *              **            *     *
//			 +      |
//			 +      |                                   ^---- There's a candle!
//			 +      +--------------------------------------------------------- angle (degrees)
//			 +
		    // TODO: adjust these thresholds per experimental data
		return findAberrantMinimums(averages, 20, 20);

}

void Robot::goSaveBaby() {
	Point direction;
	double turnAngle;
	int moveDistance = 5;//cm forwards after we're done
	for (double j = 0; j < 2 * PI; j += PI / 2) {
		if (grid[(int)(safeZoneLocation.x + cos(j) * ROBOT_DIAMETER_CM / 2 + 1)][(int)(safeZoneLocation.y + sin(j) * ROBOT_DIAMETER_CM / 2 + 1)].cellType == CLEAR);
		direction = Point((int)cos(j), (int)sin(j));
		turnAngle = (PI / 2 + j)*180/PI;
	}
	Point target = Point(safeZoneLocation.x + direction.x * ROBOT_DIAMETER_CM / 2 + 1, safeZoneLocation.y + direction.y * ROBOT_DIAMETER_CM / 2 + 1);
	createTargetPath(target);
	moveTo(moves);
	moves.clear();
	serialPrintf(arduinoSerial, "r %d\n", (int)turnAngle); // rotate
	//arduinoCommands << "r " << turnAngle << "\n";
	waitForDoneConfirmation();
	serialPrintf(arduinoSerial, "w %d\n", moveDistance);// shove it out the window
	//arduinoCommands << "w " << moveDistance << "\n";
	waitForDoneConfirmation();
	serialPrintf(arduinoSerial, "r %d\n", -(int)turnAngle);// rotate
	//arduinoCommands << "r " << -turnAngle << "\n";
	waitForDoneConfirmation();
	serialPrintf(arduinoSerial, "z 0 %d %d\n", safeZoneLocation.x + direction.x * moveDistance, safeZoneLocation.y + direction.y * moveDistance);
	currentPosCells.x = safeZoneLocation.x + direction.x * moveDistance;
	currentPosCells.y = safeZoneLocation.y + direction.y * moveDistance;
	//arduinoCommands << "z 0 " << safeZoneLocation.x + direction.x * moveDistance << " " << safeZoneLocation.y + direction.y * moveDistance << "\n";
	babySaved = true;
}

void Robot::routeToStart() {
	createTargetPath(Point(GRIDSIZE_CELLS/2, GRIDSIZE_CELLS/2));
}

double Robot::customAtan(double y, double x) {
	double temp;
	if (x == 0)
		return y < 0 ? -90 : 90;
	else
		temp = atan(y / x);
	return x < 0 ? 180 - temp : temp;
}

void Robot::outputGrid() {
	//arduinoCommands << "\n";
	for (int i = currentPosCells.x - 100; i < currentPosCells.x + 100; i++) {
		for (int j = currentPosCells.y - 100; j < currentPosCells.y + 100; j++) {
			std::cout << (grid[i][j].cellType < CLEARTHRESHOLD && grid[i][j].cellType > 0 ? 0 : ((int)grid[i][j].cellType == -1 ? 7 : (int)grid[i][j].cellType));
			//arduinoCommands << (grid[i][j].cellType < .25 ? 0 : (int)grid[i][j].cellType);
		}
		std::cout << std::endl;
	}
}

void Robot::takePicture(){
	return;
}
