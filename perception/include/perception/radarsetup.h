#include "string.h"

#define RADARFRONT_X 3.94
#define RADARFRONT_Y 0.00

#define RADARBACK_X -1.10
#define RADARBACK_Y 0.00

#define RADARLEFT_X 2.50
#define RADARLEFT_Y 1.00

#define RADARFRIGHT_X 2.50
#define RADARFRIGHT_Y -1.00

struct ObstacleType{
    std::string type;
    float width;
    float length;
};

std::vector<ObstacleType> ObstacleTypes;

void setobstalce(){
    ObstacleType obstacletype;
    ObstacleTypes.clear();
    for (int i = 1; i < 13; i++){
        if ( 1 == i ){
            obstacletype.type = "Car";
            obstacletype.width = 2.5;
            obstacletype.length = 5;  
            printf("set car \n"); 
        }

        else if ( 4 == i ){
            obstacletype.type = "Human";
            obstacletype.width = 0.5;
            obstacletype.length = 0.5;
            printf("set human \n");         
        }

        else if ( 12 == i ){
            obstacletype.type = "Building";
            obstacletype.width = 5;
            obstacletype.length = 5;    
            printf("set building \n");        
        }
        else{
            obstacletype.type = "NO_defination";
            obstacletype.width = 1;
            obstacletype.length = 1;
            printf("set others \n");
        }
        ObstacleTypes.push_back(obstacletype);
    }

    return;
}

