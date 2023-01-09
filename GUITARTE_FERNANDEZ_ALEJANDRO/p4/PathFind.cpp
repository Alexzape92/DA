// ###### Config options ################

#define PRINT_PATHS 1

// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1
               
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

#ifdef PRINT_PATHS
#include "ppm.h"
#endif

using namespace Asedio;

Vector3 cellCenterToPosition(int i, int j, float cellWidth, float cellHeight){ 
    return Vector3((j * cellWidth) + cellWidth * 0.5f, (i * cellHeight) + cellHeight * 0.5f, 0); 
}

bool operator ==(Vector3 v1, Vector3 v2){
    return (v1.x == v2.x && v1.y == v2.y && v1.z == v2.z);
}

List<AStarNode*>::const_iterator inside(AStarNode* e, const List<AStarNode*>& L){
    for(auto it = L.begin(); it != L.end(); it++){
        if ((*it)->position == e->position){
            return it;
        }
    }
    return L.end();
}

AStarNode* extrae_mejor(const List<AStarNode*>& L){
    float max = std::numeric_limits<float>::min();
    AStarNode* best = NULL;

    for(auto it = L.begin(); it != L.end(); it++){
        if((*it)->F > max){
            max = (*it)->F;
            best = *it;
        }
    }

    return best;
}

void get_path(AStarNode* last, List<Vector3>& path){
    AStarNode* i = last;
    while(i->parent != NULL){
        path.push_front(i->position);
        i = i->parent;
    }
}

void DEF_LIB_EXPORTED calculateAdditionalCost(float** additionalCost
                   , int cellsWidth, int cellsHeight, float mapWidth, float mapHeight
                   , List<Object*> obstacles, List<Defense*> defenses) {

    float cellWidth = mapWidth / cellsWidth;
    float cellHeight = mapHeight / cellsHeight;

    for(int i = 0 ; i < cellsHeight ; ++i) {
        for(int j = 0 ; j < cellsWidth ; ++j) {
            Vector3 cellPosition = cellCenterToPosition(i, j, cellWidth, cellHeight);
            
            float cost = 0;
            for(auto it = defenses.begin(); it != defenses.end(); it++){
                if(_distance(cellPosition, (*it)->position) <= (*it)->range)
                    cost += (*it)->damage / (*it)->attacksPerSecond * cellsWidth * 100;
            }
            
            additionalCost[i][j] = cost;
        }
    }
}

void DEF_LIB_EXPORTED calculatePath(AStarNode* originNode, AStarNode* targetNode
                   , int cellsWidth, int cellsHeight, float mapWidth, float mapHeight
                   , float** additionalCost, std::list<Vector3> &path) {

    int maxIter = 100;
    List<AStarNode*> abierta{}, cerrada{};

    originNode->H = _distance(originNode->position, targetNode->position);
    originNode->G = 0;
    originNode->F = originNode->G + originNode->H;
    originNode->parent = NULL;
    abierta.push_back(originNode);

    while(!abierta.empty() && path.empty() && maxIter > 0) { // @todo ensure current and target are connected
        AStarNode* k = extrae_mejor(abierta);
        abierta.remove(k);
        cerrada.push_back(k);
        if(k->position == targetNode->position){
            get_path(k, path);
        }
        else{
            for(auto it = k->adjacents.begin(); it != k->adjacents.end(); it++){
                float cost = k->G + _distance(k->position, (*it)->position);
                if(additionalCost != NULL)
                    cost += additionalCost[(int)((*it)->position.y / cellsHeight)][(int)((*it)->position.x / cellsWidth)];
                (*it)->G = cost;
                (*it)->H = _distance((*it)->position, targetNode->position);
                (*it)->F = (*it)->G + (*it)->H;
                
                auto ab = inside(*it, abierta);
                if(ab != abierta.end()){
                    if(cost < (*ab)->G)
                        abierta.remove(*ab);
                }
                auto ce = inside(*it, cerrada);
                if(ce != cerrada.end()){
                    if(cost < (*ce)->G)
                        cerrada.remove(*ce);
                }
                if(inside(*it, abierta) == abierta.end() && inside(*it, cerrada) == cerrada.end()){
                    (*it)->parent = k;
                    abierta.push_back(*it);
                }
            }
        }
        maxIter--;
    }


}
