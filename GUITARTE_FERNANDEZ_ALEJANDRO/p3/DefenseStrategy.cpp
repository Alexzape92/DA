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
//ALGORITMO SIN ORDENAR
////---------------------------------------------------------------------------------------------------

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

//---------------------------------------------------------------------------------------------------
//ORDENACION POR FUSION
//---------------------------------------------------------------------------------------------------

void fusion(std::vector<tipoCelda>& L, int i, int k, int j){
    int n = j-i+1, p = i, q = k+1;
    std::vector<tipoCelda> w;
    w.reserve(L.size());
    for(int l = 0; l < n; l++){
        if(p <= k && (q > j || L[p].value <= L[q].value)){
            w[l] = L[p];
            p++;
        }
        else{
            w[l] = L[q];
            q++;
        }
    }
    for(int l = 0; l < n; l++)
        L[i + l] = w[l];
}

void OrdenaFusion(std::vector<tipoCelda>& L, int i, int j){
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
        int k = i + n/2;
        OrdenaFusion(L, i, k);
        OrdenaFusion(L, k+1, j);
        fusion(L, i, k, j);
    }
}

std::vector<tipoCelda> getListFusion(int nCellsWidth, int nCellsHeight, bool** freecells , float mapWidth, float mapHeight, List<Object*> obstacles, List<Defense*> defenses){
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

    OrdenaFusion(L, 0, L.size()-1);
    return L;
}

void algoritmoVorazFusion(int nCellsWidth, int nCellsHeight, bool** freeCells , float mapWidth, float mapHeight, const List<Object*> obstacles, List<Defense*> defenses){
    float cellWidth = mapWidth / nCellsWidth;
    float cellHeight = mapHeight / nCellsHeight;

    List<Defense*>::iterator currentDefense = defenses.begin();
    while(currentDefense != defenses.end()) {
        std::vector<tipoCelda> C = getListFusion(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);   //Conjunto de candidatos
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

    std::make_heap(L.begin(), L.end()); //Ahora guardamos L como un montículo
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
    double t1 = c.tiempo() / r;

    c.activar();
    do {
	    algoritmoVorazFusion(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);
		
		++r;
    } while(c.tiempo() < e_abs/e_rel + e_abs);
    c.parar();
    double t2 = c.tiempo() / r;

    c.activar();
    do {
		algoritmoVorazRapida(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);
		
		++r;
    } while(c.tiempo() < e_abs/e_rel + e_abs);
    c.parar();
    double t3 = c.tiempo() / r;

    c.activar();
    do {
		algoritmoVorazMont(nCellsWidth, nCellsHeight, freeCells, mapWidth, mapHeight, obstacles, defenses);

		++r;
    } while(c.tiempo() < e_abs/e_rel + e_abs);
    c.parar();
    double t4 = c.tiempo() / r;

    std::cout << (nCellsWidth * nCellsHeight) << '\t' << t1 << '\t' << t2 << '\t' << t3 << '\t' << t4 << std::endl;
}
