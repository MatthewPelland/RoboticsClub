#ifndef POINT_H
#define POINT_H
class Point {
public:
        int x, y;

    Point() {
        x = 0;
        y = 0;
    }

    Point(int x_, int y_) {
        x = x_;
        y = y_;
    }

    static double distance(Point a, Point b) {
                return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
    }

};

#endif
