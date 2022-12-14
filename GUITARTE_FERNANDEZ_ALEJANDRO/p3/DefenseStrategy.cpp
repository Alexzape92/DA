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

    tipoCelda(Vector3 V = Vector3(), int r = 0, int c = 0, float v = 0): position{V}, row{r}, col{c}, value{v} {}
};
bool operator < (const tipoCelda& c1, const tipoCelda& c2){
    return c1.value < c2.value;
}

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


//---------------------------------------------------------------------------------------------------
//ORENACION RAPIDA
//---------------------------------------------------------------------------------------------------
int pivote(std::vector<tipoCelda>& L, int i, int j){
    int p = i;
    tipoCelda x = L[i];
    for(int k = i+1; k < j; k++){
        if(L[k].value <= x.value){
            p++;
            tipoCelda aux = L[p];
            L[p] = L[k];
            L[k] = aux;
        }
    }
    L[i] = L[p];
    L[p] = x;

    return p;
}

void OrdenaRapida(std::vector<tipoCelda>& L, int i, int j){
    int n = j-i + 1;
    if(n <= 3){
        //ORDENACION POR INSERCION
        for(int k = i; k <= j; k++){
            int pos = k;
            tipoCelda aux = L[k];
            while(pos > 0 && L[pos-1].value > aux.value){
                L[pos] = L[pos-1];
                pos--;
            }
            L[pos] = aux;
        }
    }
    else{
        int p = pivote(L, i, j);
        OrdenaRapida(L, i, p-1);
        OrdenaRapida(L, p+1, j);
    }
}

std::vector<tipoCelda> getListRapida(int nCellsWidth, int nCellsHeight, bool** freecells , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    std::vector<tipoCelda> L;
    for(int r = 0; r < nCellsHeight; r++){
        for(int c = 0; c < nCellsWidth; c++){
            float val = defaultCellValue(r, c, freecells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
            tipoCelda cell(cellCenterToPosition(r, c, cellWidth, cellHeight), r, c, val);
            L.push_back(cell); //Todo en desorden
        }
    }

    OrdenaRapida(L, 0, L.size()-1);
    return L;
}

void algoritmoVorazRapida(int nCellsWidth, int nCellsHeight, bool** freeCells , float mapWidth, float mapHeight, const List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    List<Defense*>::iterator currentDefense = defenses.begin();
    while(currentDefense != defenses.end()) {
        std::vector<tipoCelda> C = getListRapida(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);   //Conjunto de candidatos
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

//---------------------------------------------------------------------------------------------------
//MONTICULO
//---------------------------------------------------------------------------------------------------
std::vector<tipoCelda> getListMont(int nCellsWidth, int nCellsHeight, bool** freecells , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;
    std::vector<tipoCelda> L;
    for(int r = 0; r < nCellsHeight; r++){
        for(int c = 0; c < nCellsWidth; c++){
            float val = defaultCellValue(r, c, freecells, nCellsWidth, nCellsHeight, mapWidth, mapHeight, obstacles, defenses);
            tipoCelda cell(cellCenterToPosition(r, c, cellWidth, cellHeight), r, c, val);
            L.push_back(cell); //Todo en desorden
        }
    }

    std::make_heap(L.begin(), L.end()); //Ahora guardamos L como un mont??culo
    return L;
}

void algoritmoVorazMont(int nCellsWidth, int nCellsHeight, bool** freeCells , float mapWidth, float mapHeight, const List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    List<Defense*>::iterator currentDefense = defenses.begin();
    while(currentDefense != defenses.end()) {
        std::vector<tipoCelda> C = getListMont(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);   //Conjunto de candidatos
        bool solucionado = false;
        while(!solucionado && !C.empty()){
            tipoCelda p = C.front();
            std::pop_heap(C.begin(), C.end());
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

//---------------------------------------------------------------------------------------------------
//FUNCION PRINCIPAL
//---------------------------------------------------------------------------------------------------
void DEF_LIB_EXPORTED placeDefenses3(bool** freeCells, int nCellsWidth, int nCellsHeight, float mapWidth, float mapHeight
              , List<Object*> obstacles, List<Defense*> defenses) {

    if(nCellsHeight*nCellsWidth > 1000) //En los datos de la medici??n se ve que el mont??culo es mejor para n <= 1000, aproximadamente
	    algoritmoVorazRapida(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);
    else
	    algoritmoVorazMont(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);
}
