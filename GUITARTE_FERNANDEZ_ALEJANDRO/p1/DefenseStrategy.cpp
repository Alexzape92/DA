// ###### Config options ################

//#define PRINT_DEFENSE_STRATEGY 1    // generate map images

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include <limits>

#ifdef PRINT_DEFENSE_STRATEGY
#include "ppm.h"
#endif

#ifdef CUSTOM_RAND_GENERATOR
RAND_TYPE SimpleRandomGenerator::a;
#endif

using namespace Asedio;

struct tipoCelda{
    Vector3 position;
    int row, col;
    float value;

    tipoCelda(Vector3 V, int r, int c, float v): position{V}, row{r}, col{c}, value{v} {}
};

//FUNCIONES PRIVADAS
bool corta(Vector3 pos1, Vector3 pos2, float r1, float r2);
Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight);

bool corta(Vector3 pos1, Vector3 pos2, float r1, float r2){
    return r1 + r2 > _distance(pos1, pos2);
}

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ 
    return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); 
}

float cellValue0(int row, int col, float cellWidth, float cellHeight, int nCellsWidth, int nCellsHeight, List<Object*> obstacles) {
    float mino = std::numeric_limits<float>::max(), mine = mino, diste;
	for(auto o = obstacles.begin(); o != obstacles.end(); o++){
        float dist = _distance(cellCenterToPosition(row, col, cellWidth, cellHeight), (*o)->position);
        if(dist < mino)
            mino = dist;
    }

    
    mine = _distance(cellCenterToPosition(0, 0, cellWidth, cellHeight), cellCenterToPosition(row, col, cellWidth, cellHeight));

    diste = _distance(cellCenterToPosition(nCellsHeight - 1, 0, cellWidth, cellHeight), cellCenterToPosition(row, col, cellWidth, cellHeight));
    if(diste < mine)
        mine = diste;
    
    diste = _distance(cellCenterToPosition(0, nCellsWidth - 1, cellWidth, cellHeight), cellCenterToPosition(row, col, cellWidth, cellHeight));
    if(diste < mine)
        mine = diste;

    diste = _distance(cellCenterToPosition(nCellsHeight - 1, nCellsWidth - 1, cellWidth, cellHeight), cellCenterToPosition(row, col, cellWidth, cellHeight));
    if(diste < mine)
        mine = diste;
    
    return mino + mine;
}   //Esta es la que asigna valor para el centro de extraccion, crear otra para el resto

float cellValueRest(int row, int col, Defense* c0 , float cellWidth, float cellHeight){
    return _distance(c0->position, cellCenterToPosition(row, col, cellWidth, cellHeight));    //A menos distancia, mejor
}

List<tipoCelda> getList0(int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight, List<Object*> obstacles){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    List<tipoCelda> L;
    for(int r = 0; r < nCellsHeight; r++){
        for(int c = 0; c < nCellsWidth; c++){
            float val = cellValue0(r, c, cellWidth, cellHeight, nCellsWidth, nCellsHeight, obstacles);
            tipoCelda cell(cellCenterToPosition(r, c, cellWidth, cellHeight), r, c, val);
            if(L.empty())
                L.push_front(cell);
            else{
                auto p = L.begin();
                while(p != L.end() && p->value < cell.value){
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
            tipoCelda cell(cellCenterToPosition(r, c, cellWidth, cellHeight), r, c, val);
            if(L.empty())
                L.push_front(cell);
            else{
                auto p = L.begin();
                while(p != L.end() && p->value < cell.value){
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
    Vector3 posi = cellCenterToPosition(row, col, cellWidth, cellHeight);   //Posicion de nuestra nueva defensa
    if(posi.x - ((*def)->radio) < 0 || posi.x + ((*def)->radio) > mapWidth || posi.y - ((*def)->radio) < 0 || posi.y + ((*def)->radio) > mapHeight)
        res = false;
    else{
        auto d = defenses.begin();
        while(d != defenses.end() && res){  
            if(corta((*d)->position, posi, (*d)->radio, (*def)->radio)) 
                res = false;
            d++;
        }
        auto o = obstacles.begin();
        while(res && o != obstacles.end()){
            if(corta((*o)->position, posi, (*o)->radio, (*def)->radio))
                res = false;
            o++;
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
            List<tipoCelda> C = getList0(nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles);   //Conjunto de candidatos
            bool solucionado = false;
            while(!solucionado && !C.empty()){
                tipoCelda p = C.back();    //Como C está ordenado de menor a mayor valor, la funcion de selección saca el elemento en la última posición
                C.pop_back();              //Y la sacamos de la lista de candidatos
                if(factible(p.row, p.col, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, currentDefense, defenses)){
                    (*currentDefense)->position = p.position;   //Lo ponemos en la celda
                    freeCells[p.row][p.col] = false;
                    solucionado = true; //Problema solucionado
                }
            }
        }
        else{   //Colocar el resto de defensas --- Algoritmo devorador
            List<tipoCelda> C = getListRest(nCellsWidth, nCellsHeight, mapWidth, mapHeight, *(defenses.begin()));   //Conjunto de candidatos
            bool solucionado = false;
            while(!solucionado && !C.empty()){
                tipoCelda p = C.front();    //Como C está ordenado, la funcion de selección saca el elemnto en la primera posición
                C.pop_front();              //Y la sacamos de la lista de candidatos
                if(factible(p.row, p.col, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, currentDefense, defenses)){
                    (*currentDefense)->position = p.position;   //Lo ponemos en la celda
                    freeCells[p.row][p.col] = false;
                    solucionado = true; //Problema solucionado
                }
            }
        }

        ++currentDefense;
        maxAttemps--;
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
