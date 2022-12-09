#ifndef DFS_HPP_
#define DFS_HPP_

#include "../simulador/Asedio.h"
#include "../simulador/Defense.h"

using namespace Asedio;

struct tipoCelda{
    Vector3 position;
    int row, col;
    float value;

    tipoCelda(Vector3 V = Vector3(), int r = 0, int c = 0, float v = 0): position{V}, row{r}, col{c}, value{v} {}
};

void OrdenaFusion(std::vector<tipoCelda>& L, int i, int j);
void OrdenaRapida(std::vector<tipoCelda>& L, int i, int j);

#endif