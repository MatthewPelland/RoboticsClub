#include "Robot.h"
// For std::sleep
#include <thread>
#include <string>
#include <chrono>

typedef std::chrono::high_resolution_clock Clock;

//good
Robot::Robot(int level) {
	//////////////////////
	// member variables //
	//////////////////////
	this->level = level;
	this->angle = 0;
	this->sensorDist_cm = 200; //maximum readable distance
	this->angVel = 0;
	this->moves = std::vector<Point>();
	this->target = Point(0, 0);
	this->unextinguishedCandleCount = 0;
	this->inRoom = true;
	currTime = Clock::now();
	prevTime = Clock::now();
	safeZoneLocation = Point(0, 0);
	currentPosCells = Point(GRIDSIZE_CELLS / 2, GRIDSIZE_CELLS / 2);

	//////////////////////
	//  everything else //
	//////////////////////

	arduinoSerial = serialOpen("/dev/ttyACM0", 9600);
	if (level == 3) {
		this->babySaved = false;
		this->cradleFound = false;
		this->babyObtained = false;
		this->safeZoneFound = false;
	}
	else {
		this->babySaved = true;
		this->cradleFound = true;
		this->babyObtained = true;
		this->safeZoneFound = true;
	}
	for (int i = 0; i < GRIDSIZE_CELLS; i++) {
		for (int j = 0; j < GRIDSIZE_CELLS; j++) {
			grid[i][j] = gridVal();
		}
	}

    for (int i = 0; i < GRIDSIZE_CELLS; i++) {
		memset(distanceField[i], -1, sizeof(int) * GRIDSIZE_CELLS);
    }

    initialScan();//align robot
	serialPrintf(arduinoSerial, "z %d %d %d", 0, currentPosCells.x, currentPosCells.y);
}

//good
Robot::~Robot() {
    //delete grid;
    //delete distanceField;
}

//good
void Robot::update() {//this seems legit
	//this is all getting cleared out and moved into scanSurroundings
	updateTime(); //this can't go here this has to go somewhere else like only while we're moving and doing shit
	moves.clear();
	scanSurroundings();
	int mazeComplete;
	if(!babySaved)
		mazeComplete = createTargetPath(findNextTarget(true));//ignore candles
	else
		mazeComplete = createTargetPath(findNextTarget());
	if (unextinguishedCandleCount == 0 && mazeComplete) {//no unextinguished candles and maze fully mapped
		returnToStart();
	}
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

void Robot::updateTime() {
	prevTime = currTime;
	currTime = prevTime = Clock::now();
}

//good
void Robot::initialScan() {
    //can be made much faster
    double angleDelta = 0;
    std::vector<double> sonarData(360);
    serialPrintf(arduinoSerial, "r 90\n");
    while (angleDelta < 90) {
		angleDelta += updateAngle((double)std::chrono::duration_cast<std::chrono::microseconds>(currTime - prevTime).count());
		sonarData[(int)(angleDelta / (PI / 2.0) * 90) + 0 * 90] = sonic0.getDistance();
		sonarData[(int)(angleDelta / (PI / 2.0) * 90) + 1 * 90] = sonic1.getDistance();
		sonarData[(int)(angleDelta / (PI / 2.0) * 90) + 2 * 90] = sonic2.getDistance();
		sonarData[(int)(angleDelta / (PI / 2.0) * 90) + 3 * 90] = sonic3.getDistance();
		/*for (int i = 0; i < 4; i++) {
			sonarData[(int)(angleDelta / (PI / 2.0) * 90) + i * 90] = getSonarData(SONARPIN1 + i);
		}*/
    }
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
    serialPrintf(arduinoSerial, "r %d\n", mostPerpindicular-90 < 180 \
		 ? mostPerpindicular-90 \
		 : mostPerpindicular-90 - 360);
	angle = 0;
}

//needs lots of edits once camera shit is figured out
//refactor the shit out of this
void Robot::scanSurroundings(bool ignoreCandles = false) {//double check that this one is right
	/////////////////////
	//   Gather Data   //
	/////////////////////
	Point candle = Point(-1, -1);//lit Candle
	std::vector<Point> cradles = std::vector<Point>();
	std::vector<Point> windows = std::vector<Point>();
	std::vector<Point> unlitCandles = std::vector<Point>();
	double sonarData[4][360];
	for (int i = 0; i < 4; i++) {
		memset(sonarData[i], -1, sizeof(int) * 360);
	}
	int flameSensorData[360];
	memset(flameSensorData, -1, sizeof(int) * 360);
	//void cameraData[360];
	serialPrintf(arduinoSerial, "r 360\n");
	while (!serialDataAvail(arduinoSerial)) {
		updateAngle(ACCELEROMETERPIN);
		int scanAngle = int(angle * 180 / PI);
		flameSensorData[scanAngle] = flameSensor.getFireIntensity();
			sonarData[0][(int)(angle * 180 / PI + double(0) * 90) % 360] = sonic0.getDistance() + 15;
			sonarData[1][(int)(angle * 180 / PI + double(1) * 90) % 360] = sonic1.getDistance() + 15;
			sonarData[2][(int)(angle * 180 / PI + double(2) * 90) % 360] = sonic2.getDistance() + 7; //offset in by 8 cm
			sonarData[3][(int)(angle * 180 / PI + double(3) * 90) % 360] = sonic3.getDistance() + 15;
	}

	//////////////////////
	//   Analyze Data   //
	//////////////////////

	serialFlush(arduinoSerial);
	windows = checkForWindow(sonarData); //also clears out false window data, so necessary for all levels
	if (level == 3) {
		unlitCandles = locateCandles(sonarData);
		if(!babyObtained)
			cradles = checkForCradle(sonarData);
	}
	for (int sensor = 0; sensor < 4; sensor++) {
		for (int i = 0; i < 360; i++) {
			//do sonar stuff
			double distanceReading = sonarData[sensor][i];
			if (distanceReading < sensorDist_cm && distanceReading > 0) {
				int targetCellX = int((distanceReading * cos(i + 90 * sensor)) / CELLSIZE_CM + currentPosCells.x);
				int targetCellY = int((distanceReading * sin(i + 90 * sensor)) / CELLSIZE_CM + currentPosCells.y);
				gridVal* targetCell = &(grid[targetCellX][targetCellY]);
				//check fire sensor
				if (sensor == 0) {//front sensor
					if (flameSensorData[i] >= FLAMESENSORTHRESHOLD) {
						targetCell->cellType = FLAME;
						candle = Point(targetCellX, targetCellY);
					}
				}

				if (targetCell->cellType <= 1) {//in the wall/clear gradient or -1 (uninitialized)
					updateGridVal(targetCellX, targetCellY, WALL);
				}
				//increase apparent size of walls to compensate for the size of the robot 
				//so we don't have to keep track of distance to walls elsewhere.
				for (int i = -ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; i <= ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; i++) {
					for (int j = -ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; j <= ROBOT_DIAMETER_CM / CELLSIZE_CM / 2; j++) {
						gridVal* offsetTarget = &(grid[targetCellX + i][targetCellY + j]);
						if (offsetTarget->cellType <= 1) {
							updateGridVal(targetCellX + i, targetCellY + j, WALL);
						}
					}
				}
			}
			for (int i = 0; i < distanceReading; i++) {
				int cellX = int(i * cos(angle) / CELLSIZE_CM + currentPosCells.x);
				int cellY = int(i * sin(angle) / CELLSIZE_CM + currentPosCells.y);;
				updateGridVal(cellX, cellY, CLEAR);
			}
		}
	}
	// 1. lit candle
	if (candle.x != -1) {
		if (!ignoreCandles)
			extinguishCandle(candle);//immediately go put out the candle
		else
			unextinguishedCandleCount++;
	}

	//2. unlitCandles
	if (unlitCandles.size() > 0) {
		for (int i = 0; i < unlitCandles.size(); i++) {
			if (grid[unlitCandles[i].x][unlitCandles[i].y].cellType != FLAME  
				&& grid[unlitCandles[i].x][unlitCandles[i].y].cellType != CANDLE 
				&& grid[unlitCandles[i].x][unlitCandles[i].y].cellType != EXTINGUISHED) {
				grid[unlitCandles[i].x][unlitCandles[i].y].cellType == CANDLE;
				unextinguishedCandleCount++;
			}
		}
	}

	//3. window
	if (!babySaved && windows.size() > 0) { //we currently have the baby and we see a window
		for (int i = 0; i < windows.size(); i++) {
			Point direction;
			double angle;
			for (double j = 0; j < 2 * PI; j += PI / 2) {
				if (grid[(int)(windows[i].x + cos(j) * ROBOT_DIAMETER_CM / 2 + 1)][(int)(windows[i].y + sin(j) * ROBOT_DIAMETER_CM / 2 + 1)].cellType == CLEAR);
				direction = Point((int)cos(j), (int)sin(j));
				angle = PI / 2 - j;
			}
			Point target = Point(windows[i].x + direction.x * ROBOT_DIAMETER_CM / 2 + 1, windows[i].y + direction.y * ROBOT_DIAMETER_CM / 2 + 1);
			createTargetPath(target);
			moveTo(moves, true);
			//determine which way to rotate wait that information is just straight from direction, fucking idiot.
			serialPrintf(arduinoSerial, "r %d\n", angle);
			while (!serialDataAvail(arduinoSerial));
			serialFlush(arduinoSerial);
			bool isSafeZone = checkImageForSafeZone();
			if (isSafeZone) {
				serialPrintf(arduinoSerial, "w 10\n");
				while (!serialDataAvail(arduinoSerial)); //just wait for it to tell us we're finished
				serialFlush(arduinoSerial);
				serialPrintf(arduinoSerial, "r %d\n", -angle);
				babyObtained = true;
				if (safeZoneFound) {
					goToSafeZone(); //orients the robot and everything

				}
				break;
			}
			else {
				serialPrintf(arduinoSerial, "r %d\n", -angle);
			}
		}
	}

	//4. cradle
	if (!babyObtained && cradles.size() != 0) {
		//DO LOTS OF SHIT IN HERE LIKE GO SAVE THE BABY
		//make this a function dummkopf
		int moveDistance = 5;//cm after done getting baby
		for (int i = 0; i < cradles.size(); i++) {
			Point direction;
			double angle;
			for (double j = 0; j < 2 * PI; j += PI / 2) {
				if (grid[(int)(cradles[i].x + cos(j) * ROBOT_DIAMETER_CM / 2 + 1)][(int)(cradles[i].y + sin(j) * ROBOT_DIAMETER_CM / 2 + 1)].cellType == CLEAR);
				direction = Point((int)cos(j), (int)sin(j));
				angle = PI / 2 - j;
			}
			Point target = Point(cradles[i].x + direction.x * ROBOT_DIAMETER_CM / 2 + 1, cradles[i].y + direction.y * ROBOT_DIAMETER_CM / 2 + 1);
			createTargetPath(target);
			moveTo(moves, true);
			//determine which way to rotate wait that information is just straight from direction, fucking idiot.
			serialPrintf(arduinoSerial, "r %d\n", angle);
			while (!serialDataAvail(arduinoSerial));
			serialFlush(arduinoSerial);
			bool isCradle = checkImage();
			if (isCradle) {
				serialPrintf(arduinoSerial, "c %d\n", moveDistance);
				while (!serialDataAvail(arduinoSerial)); //just wait for it to tell us we're finished
				serialFlush(arduinoSerial);
				serialPrintf(arduinoSerial, "r %d\n", -angle);
				serialPrintf("z 0 %d %d\n", safeZoneLocation.x + direction.x * moveDistance, safeZoneLocation.y + direction.y * moveDistance);
				babyObtained = true;
				if (safeZoneFound) {
					goSaveBaby(); //does everything
					babySaved = true;
				}
				break;
			}
		}
	}
}

//good
void Robot::updateGridVal(int cellX, int cellY, int type) {
	grid[cellX][cellY].cellType = (grid[cellX][cellY].cellType*grid[cellX][cellY].timesScanned + type) / (grid[cellX][cellY].timesScanned++ + 1);
}

//check later
int Robot::createTargetPath(Point target, int thresholdDistance = 100) {
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
		for (int i = 0; i < openNeighbors.size(); i++) {
			//if the neighbor is one square closer to robot than the current square
			if (distanceField[openNeighbors[i].x][openNeighbors[i].y] == currentDist - 1) {
				currentDist = distanceField[openNeighbors[i].x][openNeighbors[i].y];
				prevDirection.x = currDirection.x;
				prevDirection.y = currDirection.y;
				currDirection.x = openNeighbors[i].x - currentSquare.x;
				currDirection.y = openNeighbors[i].y - currentSquare.y;

				if (prevDirection.x != currDirection.x || prevDirection.y != currDirection.y || (moves.size() == 0 && (abs(currentSquare.x - target.x) + abs(currentSquare.y - target.y)) > thresholdDistance)) {
					//if there is a change in direction
					moves.insert(moves.begin(), currentSquare);
				}
				currentSquare = openNeighbors[i];
			}
		}
	}
    
	return 0;//map not completed
}

//can be absorbed into computeDistanceField but don't screw with it.
Point Robot::findNextTarget(bool ignoreCandles = false) {//This one's pretty fast
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
		for (int i = 0; i < openNeighbors.size(); i++) {
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

//needs work
void Robot::extinguishCandle(Point target) {
	if (!inRoom) {
		createTargetPath(closestOpenCell(target), 30);
		moveTo(moves);
	}
	//actually extinguish the candle
	double angleToMove = customAtan(target.y - currentPosCells.y, target.x - currentPosCells.x);//not quite right
	serialPrintf(arduinoSerial, "r %d\n", (int)(angleToMove * 180 / PI));
	while (flameSensor.getFireIntensity() > FLAMESENSORTHRESHOLD) {
		serialPrintf(arduinoSerial, "e\n");//for extinguish
		while (!serialDataAvail(arduinoSerial));
	}
	serialPrintf(arduinoSerial, "e\n");//for extinguish
	//check if its put out, if not, continue
	//rotate back
	serialFlush(arduinoSerial);
	serialPrintf(arduinoSerial, "r %d\n", (int)(-angleToMove * 180 / PI));
	//mark all candles in the area as extinguished
	for(int i = -2; i <= 2; i ++)
		for(int j = -2; j <= 2; j++)
			if(grid[target.x + i][target.y + j].cellType == CANDLE || grid[target.x + i][target.y + j].cellType == FLAME)
				grid[target.x + i][target.y + j].cellType = EXTINGUISHED;
	while (!serialDataAvail(arduinoSerial));
	serialFlush(arduinoSerial);
}

//good
Point Robot::closestOpenCell(Point target) {//for getting to candles
	//I could also use this for getting into position for the cradle and windows
	std::vector<Point> boundary;
	boundary.push_back(Point(target.x, target.y));
	while (boundary.size() > 0) {
		Point checking = boundary[0];
		boundary.erase(boundary.begin());
		std::vector<Point> openNeighbors;
		openNeighbors = findOpenNeighbors(checking);
		for (int i = 0; i < openNeighbors.size(); i++) {
			Point current = openNeighbors[i];
			if (grid[current.x][current.y].cellType <= CLEARTHRESHOLD) {
				return current;
			}
			else {
				if (distance(target, current) > distance(target, checking)) {
					boundary.push_back(current);
				}
			}
		}
	}
}

//good
double Robot::distance(Point a, Point b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

//I mean that's pretty much it, no?
void Robot::pushBabyOutWindow() {
	serialPrintf(arduinoSerial, "w\n");
	babySaved = true;
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
		for (int i = 0; i < openNeighbors.size(); i++) {
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
			grid[currentPos.x + x_offset][currentPos.y + y_offset].cellType <= CLEARTHRESHOLD) {
			openNeighbors.push_back(Point(currentPos.x + x_offset,
							  currentPos.y + y_offset));
			}
		}
    }
    return openNeighbors;
}

//good
/*
x\y 1  0 -1
1   n     n
0      X
-1  n     n
*/
bool is_diagonal_candidate(int x_offset, int y_offset) {
	return ((x_offset + y_offset + 2) % 2 == 0);
}

//add code for waiting and updating
void Robot::moveTo(std::vector<Point> moves, bool takePictures = false) {
    // send commands to Arduino to go somewhere by grid coordinates
	if (moves.size() >= 0) {
		serialPrintf(arduinoSerial, "m ");//m for moves
		for (int i = 0; i < moves.size(); i++) {
			serialPrintf(arduinoSerial, "%d %d ", moves[i].x, moves[i].y);
		}
		serialPrintf(arduinoSerial, "\n");
	}
	while (!serialDataAvail(arduinoSerial)) 
		if(takePictures)
			takePicture();
	char receive[15];
	int i = 0;
	while (receive[i] = serialGetChar(arduinoSerial) != '\n')
		i++;
	sscanf(receive, "%d", &inRoom);
	sscanf(receive, "%d", &(currentPosCells.x));
	sscanf(receive, "%d", &(currentPosCells.y));
}

double Robot::updateAngle(double timeDelta) {
    double angVel = getAngularVelocity();//what units does this return
    double angleDelta = angVel * timeDelta/1000000;//microseconds
    angle += angleDelta;
	return angleDelta;
}

//needs that gyro code
double Robot::getAngularVelocity() {
	return sensor.getGyroZ();
}

//doesn't work at all
///      *
//      *
//   ***  
//would be read as one continuous edge how we have it right now
std::vector<Point> Robot::checkForCradle(double sonarData[4][360]) {
	double averages[360];
	double length = 0;
	std::vector<Point> midpoints;

	for (int i = 0; i < 4; i++) { //find averages
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
		DoublePoint p1 = DoublePoint(averages[(i + 360)%360] * cos(i * PI / 180), averages[(i+360)%360] * sin(i * PI / 180));
		DoublePoint p2 = DoublePoint(averages[(i + 361)%360] * cos((i + 1) * PI / 180), averages[(i + 361)%360] * sin((i + 1) * PI / 180));

		double currentLen = sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2)); //length of current vector

																				 //angle between last vector and currernt vector
		double angle = acos(((lastP2.x - lastP1.x) * (p2.x - p1.x) + (lastP2.y - lastP1.y) * (p2.y - p1.y)) / (lastLen * currentLen));

		if (abs(angle * (180 / PI)) < 60) {
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

//maybe return a vector of poinits or something in case we have false positive and miss real window
//yup we gotta do that
//also if there is a window change the values to be normal
//sonar 2 is rear, lifted up higher so as to notice the windows
std::vector<Point> Robot::checkForWindow(double sonarData[4][360]) {
	std::vector<Point> windows;
	double average;
	Point window;
	for (int i = 0; i < 360; i++) {
		average = (sonarData[1][i] + sonarData[0][i] + sonarData[3][i]) / 3;
		if ((sonarData[2][i] - average) > 10){
			//we found a window!!!
			sonarData[2][i] = average;
			windows.push_back(Point(average*cos(i*PI / 180), average*sin(i*PI / 180)));
		}
	}
	return windows;//will be changed
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
		    /*
			 +      distance  from bot
			 +      | *                               * *
			 +      |   *                          *      *
			 +      |      *                    *      **    *
			 +      |         *              *                  *
			 +      |                                                          *
			 +      |             *  .  *              **            *     *
			 +      |
			 +      |                                   ^---- There's a candle!
			 +      +--------------------------------------------------------- angle (degrees)
			 +     */
		    // TODO: adjust these thresholds per experimental data
		return findAberrantMinimums(averages, 20, 20);
	
}

//all of these functions are exactly the same
void Robot::goSaveBaby() {
	Point direction;
	double angle;
	int moveDistance = 5;//cm forwards after we're done
	for (double j = 0; j < 2 * PI; j += PI / 2) {
		if (grid[(int)(safeZoneLocation.x + cos(j) * ROBOT_DIAMETER_CM / 2 + 1)][(int)(safeZoneLocation.y + sin(j) * ROBOT_DIAMETER_CM / 2 + 1)].cellType == CLEAR);
		direction = Point((int)cos(j), (int)sin(j));
		angle = PI / 2 - j;
	}
	Point target = Point(safeZoneLocation.x + direction.x * ROBOT_DIAMETER_CM / 2 + 1, safeZoneLocation.y + direction.y * ROBOT_DIAMETER_CM / 2 + 1);
	createTargetPath(target);
	moveTo(moves, true);
	//determine which way to rotate wait that information is just straight from direction, fucking idiot.
	serialPrintf(arduinoSerial, "r %d\n", angle); // rotate
	while (!serialDataAvail(arduinoSerial));      // wait
	serialFlush(arduinoSerial);                   
	serialPrintf(arduinoSerial, "w %d\n", moveDistance);         // shove it out the window
	while (!serialDataAvail(arduinoSerial));      // wait
	serialPrintf(arduinoSerial, "r %d\n", -angle);// rotate
	while (!serialDataAvail(arduinoSerial));      // wait
	serialFlush(arduinoSerial); 
	serialPrintf("z 0 %d %d\n", safeZoneLocation.x + direction.x * moveDistance, safeZoneLocation.y + direction.y * moveDistance);
}

void Robot::returnToStart() { //route robot back to start, extinguishing any remaining candles on the way there
					//the initial position is simly stored at (0, 0) in the gridMap.
	createTargetPath(Point(GRIDSIZE_CELLS/2, GRIDSIZE_CELLS/2));
}

double customAtan(double y, double x) {
	double temp;
	if (x == 0)
		return y < 0 ? -90 : 90;
	else
		temp = atan(y / x);
	return x < 0 ? 180 - temp : temp;
}