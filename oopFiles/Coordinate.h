//
//  Point.h
//  gps
//
//  Created by Hadi Zayer on 4/7/17.
//  Copyright © 2017 Hadi Zayer. All rights reserved.
//

#ifndef Coordinate_h
#define Coordinate_h

struct Coordinate {
    Coordinate();
    Coordinate(double, double);
    
    double lattitude, longitude;
};

Coordinate(){
    self.lattitude = 0;
    self.longitude = 0;
}

Coordinate(double lat, double lon){
    self.lattitude = lat;
    self.longitude = lon;
}
#endif /* Point_h */
