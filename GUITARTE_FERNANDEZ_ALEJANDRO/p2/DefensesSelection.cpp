// ###### Config options ################


// #######################################

#define BUILDING_DEF_STRATEGY_LIB 1
               
#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

float value(Defense* def){
    return def->damage/def->attacksPerSecond + 0*def->dispersion + 0.6*def->health + 0.1*def->range;
}

int indice(std::list<Defense*>::const_iterator i, const std::list<Defense*>& defs){
    int index = 0;
    for(auto aux = defs.begin(); aux != i; aux++){
        index++;
    }

    return index;
}

Defense* defensa(int indice, const std::list<Defense*> defenses){
    auto d = defenses.begin();  //indice 0
    for(int i = 0; i < indice; i++)
        d++;
    
    return (*d);
}

void matriz(float** mat, unsigned int c, const std::list<Defense*>& defenses){
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

void recuperar(float** mat, std::list<int>& resultado, int n, int c, const std::list<Defense*>& defenses){
    int i = n-1, j = c;

    while(i != 0){
        if(mat[i][j] != mat[i-1][j]){
            resultado.push_front(defensa(i, defenses)->id);
            j -= defensa(i,defenses)->cost;
        }
        i--;
    }
    
    if(j >= (*defenses.begin())->cost)
        resultado.push_front((*defenses.begin())->id);
}

void DEF_LIB_EXPORTED selectDefenses(std::list<Defense*> defenses, unsigned int ases, std::list<int> &selectedIDs
            , float mapWidth, float mapHeight, std::list<Object*> obstacles) {

    int n = defenses.size();
    unsigned int c = ases - (*defenses.begin())->cost;  //Restamos el coste de la mina

    selectedIDs.push_front((*defenses.begin())->id);
    defenses.pop_front();   //No debemos tener en cuenta la mina
    n--;

    float** tabla = new float*[n];
    for(int i = 0; i < n; i++)
        tabla[i] = new float[c+1];

    matriz(tabla, c, defenses);  //Rellenamos la tabla
    recuperar(tabla, selectedIDs, n, c, defenses); //Guardamos en selectedIDs las IDs seleccionadas
}
