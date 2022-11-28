// ###### Config options ################



// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"
#include "cronometro.h"

using namespace Asedio;  

struct tipoCelda{
    Vector3 position;
    int row, col;
    float value;

    tipoCelda(Vector3 V, int r, int c, float v): position{V}, row{r}, col{c}, value{v} {}
};

bool corta(Vector3 pos1, Vector3 pos2, float r1, float r2);
Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight);

bool corta(Vector3 pos1, Vector3 pos2, float r1, float r2){
    return r1 + r2 > _distance(pos1, pos2);
}

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ 
    return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); 
}

float defaultCellValue(int row, int col, bool** freeCells, int nCellsWidth, int nCellsHeight               
    , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses) {
    	
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    Vector3 cellPosition((col * cellWidth) + cellWidth * 0.5f, (row * cellHeight) + cellHeight * 0.5f, 0);
    	
    float val = 0;
    for (List<Object*>::iterator it=obstacles.begin(); it != obstacles.end(); ++it) {
	    val += _distance(cellPosition, (*it)->position);
    }

    return val;
}

List<tipoCelda> getList(int nCellsWidth, int nCellsHeight, bool** freecells , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    List<tipoCelda> L;
    for(int r = 0; r < nCellsHeight; r++){
        for(int c = 0; c < nCellsWidth; c++){
            float val = defaultCellValue(r, c, freecells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
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

void algoritmoVorazSinOrdenar(int nCellsWidth, int nCellsHeight, bool** freeCells , float mapWidth, float mapHeight, const List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    List<Defense*>::iterator currentDefense = defenses.begin();
    while(currentDefense != defenses.end()) {
        List<tipoCelda> C = getList(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);   //Conjunto de candidatos
        bool solucionado = false;
        while(!solucionado && !C.empty()){
            tipoCelda p = C.back();    
            C.pop_back();              
            if(factible(p.row, p.col, freeCells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, currentDefense, defenses)){
                (*currentDefense)->position = p.position;   //Lo ponemos en la celda
                freeCells[p.row][p.col] = false;
                solucionado = true; //Problema solucionado
            }
        }
        currentDefense++;
    }
}

void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , List<Object*> obstacles, List<Defense*> defenses) {

    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight; 

	cronometro c;
    long int r = 0;
    double e_abs = 0.01, e_rel = 0.001;
    c.activar();
    do {
		algoritmoVorazSinOrdenar(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);
		
		++r;
    } while(c.tiempo() < e_abs/e_rel + e_abs);
    c.parar();

    std::cout << (nCellsWidth * nCellsHeight) << '\t' << c.tiempo() / r << std::endl;
}
