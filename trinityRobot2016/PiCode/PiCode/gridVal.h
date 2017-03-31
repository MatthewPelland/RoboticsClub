#ifndef GRIDVAL_H
#define GRIDVAL_H
struct gridVal {
        int timesScanned;
        double cellType;
        gridVal() {
                timesScanned = 0;
                cellType = -1;
        }
};

#endif
