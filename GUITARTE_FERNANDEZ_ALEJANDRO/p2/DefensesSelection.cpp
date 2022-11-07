// ###### Config options ################


// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1
               
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

float value(Defense* def){
    return def->damage + def->attacksPerSecond + def->dispersion + def->health + def->radio;
}

int indice(std::list<Defense*>::const_iterator i, const std::list<Defense*>& defs){
    int index = 0;
    for(auto aux = defs.begin(); aux != i; aux++){
        index++;
    }

    return index;
}

void matriz(float** mat, int c, const std::list<Defense*>& defenses){
    int n = defenses.size();
    auto i = defenses.begin();

    for(int j = 0; j <= c; j++){
        if(j < (*i)->cost)
            mat[0][j] = 0;
        else
            mat[0][j] = value(*i);
    }

    for(++i; i != defenses.end(); i++){
        for(int j = 0; j <= c; j++){
            if(j < (*i)->cost)
                mat[indice(i, defenses)][j] = mat[indice(i, defenses)-1][j];
            else
                mat[indice(i, defenses)][j] = 
                std::max(mat[indice(i, defenses)-1][j], mat[indice(i, defenses)-1][j-(*i)->cost] + value(*i));
        }
    }
}

void DEF_LIB_EXPORTED selectDefenses(std::list<Defense*> defenses, unsigned int ases, std::list<int> &selectedIDs
            , float mapWidth, float mapHeight, std::list<Object*> obstacles) {

    unsigned int cost = 0;
    std::list<Defense*>::iterator it = defenses.begin();
    while(it != defenses.end()) {
        
    }
}
