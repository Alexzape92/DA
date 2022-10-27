// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

typedef struct tipoCelda{
    Vector3 position;
    int row, col;
    float value;

    tipoCelda(Vector3 V, int r, int c, float v): position{V}, row{r}, col{c}, value{v} {}
};

Vector3 centro(int col, int row, float cellWidth, float cellHeight){
    return Vector3(col * cellWidth + cellWidth / 2, row * cellHeight + cellHeight/2, 0);
}

float distancia(Vector3 p1, Vector3 p2){
    return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2));
}

float cellValue0(int row, int col, int nCellsWidth, int nCellsHeight) {
	int r = std::min(std::abs(row - (nCellsHeight-1)), row);
    int c = std::min(std::abs(col - (nCellsWidth-1)), col);
    return r + c;   //Las esquinas tendrán máximo valor (0), y el centro el mínimo 
}   //Esta es la que asigna valor para el centro de extraccion, crear otra para el resto

float cellValueRest(int row, int col, Defense* c0 , float cellWidth, float cellHeight){
    return distancia(c0->position, centro(col, row, cellWidth, cellHeight));    //A menos distancia, mejor
}

List<tipoCelda> getList0(int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    List<tipoCelda> L;
    for(int r = 0; r < nCellsHeight; r++){
        for(int c = 0; c < nCellsWidth; c++){
            float val = cellValue0(r, c, nCellsWidth, nCellsHeight);
            tipoCelda cell(centro(c, r, cellWidth, cellHeight), r, c, val);
            if(L.empty())
                L.push_front(cell);
            else{
                auto p = L.begin();
                while(p != L.end() && p->value > cell.value){
                    p++;
                }
                L.insert(p, cell);
            }
        }
    }
    return L;
}

List<tipoCelda> getListRest(int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, Defense* c0){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    List<tipoCelda> L;
    for(int r = 0; r < nCellsHeight; r++){
        for(int c = 0; c < nCellsWidth; c++){
            float val = cellValueRest(r, c, c0, cellWidth, cellHeight);
            tipoCelda cell(centro(c, r, cellWidth, cellHeight), r, c, val);
            if(L.empty())
                L.push_front(cell);
            else{
                auto p = L.begin();
                while(p != L.end() && p->value > cell.value){
                    p++;
                }
                L.insert(p, cell);
            }
        }
    }
    return L;
}

bool factible(int row, int col, bool** freeCells, int nCellsWidth, int nCellsHeight
	, float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*>::iterator def, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    bool res = true;
    if(row < 0 || col < 0 || row >= nCellsHeight || col >= nCellsWidth || !freeCells[row][col])
        res = false;
    else{
        Vector3 v = centro(col, row, cellWidth, cellHeight);   //Posición de nuestra nueva defensa
        auto d = defenses.begin();
        while(d != def && res){  //Las defensas se isertan desde 0 de una en una, por lo que las siguientes a i no estan colocadas
            if(corta((*d)->position, v, (*d)->radio, (*def)->radio)) 
                res = false;
        }
        auto o = obstacles.begin();
        while(res && o != obstacles.end()){
            if(corta((*o)->position, v, (*o)->radio, (*def)->radio))
                res = false;
        }
    }
    return res;
}

void DEF_LIB_EXPORTED placeDefenses(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , std::list<Object*> obstacles, std::list<Defense*> defenses) {

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    int maxAttemps = 1000;
    List<Defense*>::iterator currentDefense = defenses.begin();
    while(currentDefense != defenses.end() && maxAttemps > 0) {
        if(currentDefense == defenses.begin()){ //Colocar centro de extracción --- Algoritmo devorador
            List<tipoCelda> C = getList0(nCellsWidth, nCellsHeight, mapWidth, mapHeight);   //Conjunto de candidatos
            bool solucionado = false;
            while(!solucionado && !C.empty()){
                tipoCelda p = C.front();    //Como C está ordenado, la funcion de selección saca el elemnto en la primera posición
                C.pop_front();              //Y la sacamos de la lista de candidatos
                if(factible(p.row, p.col, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, currentDefense, defenses)){
                    (*currentDefense)->position = p.position;   //Lo ponemos en la celda
                    solucionado = true; //Problema solucionado
                }
            }
        }
        else{   //Colocar centro de extracción --- Algoritmo devorador
            List<tipoCelda> C = getListRest(nCellsWidth, nCellsHeight, mapWidth, mapHeight, *(defenses.begin()));   //Conjunto de candidatos
            bool solucionado = false;
            while(!solucionado && !C.empty()){
                tipoCelda p = C.front();    //Como C está ordenado, la funcion de selección saca el elemnto en la primera posición
                C.pop_front();              //Y la sacamos de la lista de candidatos
                if(factible(p.row, p.col, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, currentDefense, defenses)){
                    (*currentDefense)->position = p.position;   //Lo ponemos en la celda
                    solucionado = true; //Problema solucionado
                }
            }
        }

        ++currentDefense;
    }

#ifdef PRINT_DEFENSE_STRATEGY

    float** cellValues = new float* [nCellsHeight]; 
    for(int i = 0; i < nCellsHeight; ++i) {
       cellValues[i] = new float[nCellsWidth];
       for(int j = 0; j < nCellsWidth; ++j) {
           cellValues[i][j] = ((int)(cellValue(i, j))) % 256;
       }
    }
    dPrintMap("strategy.ppm", nCellsHeight, nCellsWidth, cellHeight, cellWidth, freeCells
                         , cellValues, std::list<Defense*>(), true);

    for(int i = 0; i < nCellsHeight ; ++i)
        delete [] cellValues[i];
	delete [] cellValues;
	cellValues = NULL;

#endif
}

bool corta(Vector3 pos1, Vector3 pos2, float r1, float r2){
    return r1 + r2 > distancia(pos1, pos2);
}
