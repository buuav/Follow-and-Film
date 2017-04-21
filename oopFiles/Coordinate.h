#ifndef Coordinate_h
#define Coordinate_h

struct Coordinate {
    Coordinate(){
        lattitude = 0;
        longitude = 0;
    }
    Coordinate(double lat, double lon){
        lattitude = lat;
        longitude = lon;
    }
    
    double lattitude, longitude;
};

#endif /* Point_h */
