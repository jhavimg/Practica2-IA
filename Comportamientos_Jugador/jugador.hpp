#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

struct stateN0
{
  ubicacion jugador;
  ubicacion sonambulo;

  bool zapas_j = false, bikini_j = false, zapas_s = false, bikini_s = false;

  bool operator==(const stateN0 &x) const
  {
    if (jugador == x.jugador && sonambulo.f == x.sonambulo.f && sonambulo.c == x.sonambulo.c)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

struct nodeN0
{
  stateN0 st;
  list<Action> secuencia;

  bool operator==(const nodeN0 &n) const
  {
    return (st == n.st);
  }

  bool operator<(const nodeN0 &n) const
  {
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c == n.st.sonambulo.c && st.sonambulo.brujula < n.st.sonambulo.brujula)
      return true;
    else if (st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c < n.st.sonambulo.c)
      return true;
    else if (st.sonambulo.f < n.st.sonambulo.f)
      return true;
    else
      return false;
  }
};

struct nodeN02
{

  stateN0 st;
  list<Action> secuencia;

  int g = 0, h = 0;

  bool operator<(const nodeN02 &n) const
  {
    if (st.jugador.f < n.st.jugador.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c < n.st.jugador.c)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula < n.st.jugador.brujula)
      return true;
    else if (st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c == n.st.sonambulo.c && st.sonambulo.brujula < n.st.sonambulo.brujula)
      return true;
    else if (st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c < n.st.sonambulo.c)
      return true;
    else if (st.sonambulo.f < n.st.sonambulo.f)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula == n.st.jugador.brujula && st.bikini_j > n.st.bikini_j)
      return true;
    else if (st.jugador.f == n.st.jugador.f && st.jugador.c == n.st.jugador.c && st.jugador.brujula == n.st.jugador.brujula && st.bikini_j == n.st.bikini_j && st.zapas_j > n.st.zapas_j)
      return true;
    else if(st.sonambulo.f == n.st.sonambulo.f && st.jugador.c == n.st.sonambulo.c && st.sonambulo.brujula == n.st.sonambulo.brujula && st.bikini_s > n.st.bikini_s)
      return true;
    else if(st.sonambulo.f == n.st.sonambulo.f && st.sonambulo.c == n.st.sonambulo.c && st.sonambulo.brujula == n.st.sonambulo.brujula && st.bikini_s == n.st.bikini_s && st.zapas_s > n.st.zapas_s)
      return true;
    else
      return false;
  }

  int coste() const
  {
    return g + h;
  }
};

struct comparacion
{

  bool operator()(const nodeN02 &n1, const nodeN02 &n2) const
  {

    bool comp = false;
    if (n1.coste() > n2.coste())
      comp = true;

    return comp;
  }
};

class ComportamientoJugador : public Comportamiento
{
public:
  ComportamientoJugador(unsigned int size) : Comportamiento(size)
  {
    // Inicializar Variables de Estado
  }
  ComportamientoJugador(std::vector<std::vector<unsigned char>> mapaR) : Comportamiento(mapaR)
  {
    // Inicializar Variables de Estado
  }
  ComportamientoJugador(const ComportamientoJugador &comport) : Comportamiento(comport) {}
  ~ComportamientoJugador() {}

  Action think(Sensores sensores);
  int interact(Action accion, int valor);

private:
  // Declarar Variables de Estado
  list<Action> plan, plan2;
  bool hayPlan;
  stateN0 c_state;
  ubicacion goal;

  void VisualizaPlan(const stateN0 &st, const list<Action> &plan);

  // Nivel 0
  list<Action> AnchuraSoloJugador_V3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

  // Nivel 1

  bool ve_sonambulo(const stateN0 &st, const vector<vector<unsigned char>> &mapa);

  list<Action> anchuraNivel1(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);
  list<Action> anchuraJugador_V4(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);
  list<Action> anchuraSonambulo(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);

  // Nivel 2
  list<Action> algoritmoDijkstra(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);
  int coste(Action accion, const stateN0 &st, const vector<vector<unsigned char>> &mapa);

  // Nivel 3
  list<Action> algoritmoNivel3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);
  list<Action> sonambuloNivel3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);
  list<Action> jugadorNivel3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa);
  int distancia_manhattan(const ubicacion ubi, const ubicacion &final);
  int coste_s(Action accion, const stateN0 &st, const vector<vector<unsigned char>> &mapa);
};

#endif