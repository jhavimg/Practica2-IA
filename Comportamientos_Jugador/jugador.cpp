#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>

bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa);
ubicacion NextCasilla(const ubicacion &pos);
stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa);
bool Find(const stateN0 &item, const list<nodeN0> &lista);
void AnularMatriz(vector<vector<unsigned char>> &matriz);

Action ComportamientoJugador::think(Sensores sensores)
{
	Action accion = actIDLE;

	if (sensores.nivel != 4)
	{
		if (!hayPlan)
		{
			cout << "Calculando un nuevo plan\n";
			c_state.jugador.f = sensores.posF;
			c_state.jugador.c = sensores.posC;
			c_state.jugador.brujula = sensores.sentido;
			c_state.sonambulo.f = sensores.SONposF;
			c_state.sonambulo.c = sensores.SONposC;
			c_state.sonambulo.brujula = sensores.SONsentido;
			goal.f = sensores.destinoF;
			goal.c = sensores.destinoC;

			switch (sensores.nivel)
			{
			case 0:
				plan = AnchuraSoloJugador_V3(c_state, goal, mapaResultado);
				break;
			case 1:
				plan = anchuraNivel1(c_state, goal, mapaResultado);
				break;
			case 2:
				plan = algoritmoDijkstra(c_state, goal, mapaResultado);
				break;
			case 3:
				plan = algoritmoNivel3(c_state, goal, mapaResultado);
				break;
			}

			if (plan.size() > 0)
			{
				VisualizaPlan(c_state, plan);
				hayPlan = true;
			}
		}
		if (hayPlan && plan.size() > 0)
		{
			cout << "Ejecutando la siguiente accion del plan\n";
			accion = plan.front();
			plan.pop_front();
		}
		if (plan.size() == 0)
		{
			cout << "Se completó el plan\n";
			hayPlan = false;
		}
	}
	else
	{
		// Incluir aquí la solución al nivel 4
	}

	return accion;
}

int ComportamientoJugador::interact(Action accion, int valor)
{
	return false;
}

list<Action> ComportamientoJugador::AnchuraSoloJugador_V3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN0 current_node;
	list<nodeN0> frontier;
	plan = current_node.secuencia;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;
	bool SolutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actrFORWRAD
		nodeN0 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (child_forward.st.jugador.f == final.f && child_forward.st.jugador.c == final.c)
		{
			child_forward.secuencia.push_back(actFORWARD);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actFORWARD);
			frontier.push_back(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hojo actTRURN_L
			nodeN0 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN0 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}

	if (SolutionFound)
		plan = current_node.secuencia;

	return plan;
}

bool CasillaTransitable(const ubicacion &x, const vector<vector<unsigned char>> &mapa)
{
	return (mapa[x.f][x.c] != 'P' && mapa[x.f][x.c] != 'M');
}

ubicacion NextCasilla(const ubicacion &pos)
{
	ubicacion next = pos;
	switch (pos.brujula)
	{
	case norte:
		next.f = pos.f - 1;
		break;
	case noreste:
		next.f = pos.f - 1;
		next.c = pos.c + 1;
		break;
	case este:
		next.c = pos.c + 1;
		break;
	case sureste:
		next.f = pos.f + 1;
		next.c = pos.c + 1;
		break;
	case sur:
		next.f = pos.f + 1;
		break;
	case suroeste:
		next.f = pos.f + 1;
		next.c = pos.c - 1;
		break;
	case oeste:
		next.c = pos.c - 1;
		break;
	case noroeste:
		next.f = pos.f - 1;
		next.c = pos.c - 1;
		break;
	default:
		break;
	}

	return next;
}

stateN0 apply(const Action &a, const stateN0 &st, const vector<vector<unsigned char>> &mapa)
{
	stateN0 st_result = st;
	ubicacion sig_ubicacion;
	switch (a)
	{
	case actFORWARD: // si proximo casiila es transitable y no está ocupada por el sonámbulo
		sig_ubicacion = NextCasilla(st.jugador);
		if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == st.sonambulo.f && sig_ubicacion.c == st.sonambulo.c))
		{
			st_result.jugador = sig_ubicacion;
		}
		break;
	case actTURN_L:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 6) % 8);
		break;
	case actTURN_R:
		st_result.jugador.brujula = static_cast<Orientacion>((st_result.jugador.brujula + 2) % 8);
		break;
	case actSON_FORWARD:
		sig_ubicacion = NextCasilla(st.sonambulo);
		if (CasillaTransitable(sig_ubicacion, mapa) && !(sig_ubicacion.f == st.jugador.f && sig_ubicacion.c == st.jugador.c))
		{
			st_result.sonambulo = sig_ubicacion;
		}
		break;
	case actSON_TURN_SL:
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 7) % 8);
		break;
	case actSON_TURN_SR:
		st_result.sonambulo.brujula = static_cast<Orientacion>((st_result.sonambulo.brujula + 1) % 8);
		break;
	}

	return st_result;
}

bool Find(const stateN0 &item, const list<nodeN0> &lista)
{
	auto it = lista.begin();
	while (it != lista.end() && !((it->st == item)))
		it++;

	return (!(it == lista.end()));
}

void AnularMatriz(vector<vector<unsigned char>> &matriz)
{
	for (int i = 0; i < matriz.size(); i++)
	{
		for (int j = 0; j < matriz[0].size(); j++)
		{
			matriz[i][j] = 0;
		}
	}
}

void ComportamientoJugador::VisualizaPlan(const stateN0 &st, const list<Action> &plan)
{

	AnularMatriz(mapaConPlan);
	stateN0 cst = st;

	auto it = plan.begin();
	while (it != plan.end())
	{
		switch (*it)
		{
		case actFORWARD:
			cst.jugador = NextCasilla(cst.jugador);
			mapaConPlan[cst.jugador.f][cst.jugador.c] = 1;
			break;
		case actTURN_R:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 2) % 8);
			break;
		case actTURN_L:
			cst.jugador.brujula = (Orientacion)((cst.jugador.brujula + 6) % 8);
			break;
		case actSON_FORWARD:
			cst.sonambulo = NextCasilla(cst.sonambulo);
			mapaConPlan[cst.sonambulo.f][cst.sonambulo.c] = 2;
			break;
		case actSON_TURN_SR:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 1) % 8);
			break;
		case actSON_TURN_SL:
			cst.sonambulo.brujula = (Orientacion)((cst.sonambulo.brujula + 7) % 8);
			break;
		}
		it++;
	}
}

list<Action> ComportamientoJugador::anchuraNivel1(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{

	nodeN0 current_node;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;

	bool SolutionFound = (current_node.st.sonambulo.f == final.f && current_node.st.sonambulo.c == final.c);
	frontier.push_back(current_node);

	list<Action> jugador;
	jugador = anchuraJugador_V4(current_node.st, current_node.st.sonambulo, mapa);

	for (auto it = jugador.begin(); it != jugador.end(); ++it)
	{
		current_node.secuencia.push_back(*it);
		current_node.st = apply(*it, current_node.st, mapa);
	}

	list<Action> sonam;
	sonam = anchuraSonambulo(current_node.st, final, mapa);

	while (!sonam.empty())
	{

		current_node.secuencia.push_back(sonam.front());
		current_node.st = apply(sonam.front(), current_node.st, mapa);
		sonam.pop_front();

		if (!ve_sonambulo(current_node.st, mapa) && !sonam.empty())
		{

			list<Action> jugador2;
			jugador2 = anchuraJugador_V4(current_node.st, current_node.st.sonambulo, mapa);

			for (auto it = jugador2.begin(); it != jugador2.end(); ++it)
			{
				current_node.secuencia.push_back(*it);
				current_node.st = apply(*it, current_node.st, mapa);
			}
		}
	}

	plan = current_node.secuencia;

	return plan;
}

bool ComportamientoJugador::ve_sonambulo(const stateN0 &st, const vector<vector<unsigned char>> &mapa)
{

	bool lo_ve = false;

	float vf[15] = {-1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3};
	float vc[15] = {-1, 0, 1, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3};

	float vf2[15] = {-1, 0, 1, -2, -1, 0, 1, 2, -3, -2, -1, 0, 1, 2, 3};
	float vc2[15] = {1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3};

	float vf4[15] = {1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3};
	float vc4[15] = {1, 0, -1, 2, 1, 0, -1, -2, 3, 2, 1, 0, -1, -2, -3};

	float vf6[15] = {1, 0, -1, 2, 1, 0, -1, -2, 3, 2, 1, 0, -1, -2, -3};
	float vc6[15] = {-1, -1, -1, -2, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3};

	switch (st.jugador.brujula)
	{
	case 0:

		for (int i = 0; i < 15 && !lo_ve; i++)
		{
			if (st.jugador.f + vf[i] == st.sonambulo.f && st.jugador.c + vc[i] == st.sonambulo.c)
				lo_ve = true;
		}
		break;
	case 2:

		for (int i = 0; i < 15 && !lo_ve; i++)
		{
			if (st.jugador.f + vf2[i] == st.sonambulo.f && st.jugador.c + vc2[i] == st.sonambulo.c)
				lo_ve = true;
		}
		break;
	case 4:

		for (int i = 0; i < 15 && !lo_ve; i++)
		{
			if (st.jugador.f + vf4[i] == st.sonambulo.f && st.jugador.c + vc4[i] == st.sonambulo.c)
				lo_ve = true;
		}
		break;
	case 6:

		for (int i = 0; i < 15 && !lo_ve; i++)
		{
			if (st.jugador.f + vf6[i] == st.sonambulo.f && st.jugador.c + vc6[i] == st.sonambulo.c)
				lo_ve = true;
		}
		break;
	}

	return lo_ve;
}

list<Action> ComportamientoJugador::anchuraJugador_V4(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{

	nodeN0 current_node;
	list<nodeN0> frontier;
	plan = current_node.secuencia;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;
	bool SolutionFound = (ve_sonambulo(current_node.st, mapa));
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound)
	{
		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWRAD
		nodeN0 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (ve_sonambulo(child_forward.st, mapa))
		{
			child_forward.secuencia.push_back(actFORWARD);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actFORWARD);
			frontier.push_back(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hojo actTRURN_L
			nodeN0 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN0 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}

	if (SolutionFound)
		plan = current_node.secuencia;

	return plan;
}

list<Action> ComportamientoJugador::anchuraSonambulo(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{

	nodeN0 current_node;
	list<nodeN0> frontier;
	set<nodeN0> explored;
	list<Action> plan;
	current_node.st = inicio;

	bool SolutionFound = (current_node.st.sonambulo.f == final.f && current_node.st.sonambulo.c == final.c);
	frontier.push_back(current_node);

	while (!frontier.empty() && !SolutionFound)
	{

		frontier.pop_front();
		explored.insert(current_node);

		// Generar hijo actFORWRAD
		nodeN0 child_forward = current_node;
		child_forward.st = apply(actSON_FORWARD, current_node.st, mapa);
		if (child_forward.st.sonambulo.f == final.f && child_forward.st.sonambulo.c == final.c)
		{
			child_forward.secuencia.push_back(actSON_FORWARD);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actSON_FORWARD);
			frontier.push_back(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hijo actTRURN_L
			nodeN0 child_turnl = current_node;

			child_turnl.st = apply(actSON_TURN_SL, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actSON_TURN_SL);
				frontier.push_back(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN0 child_turnr = current_node;
			child_turnr.st = apply(actSON_TURN_SR, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actSON_TURN_SR);
				frontier.push_back(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
		{
			current_node = frontier.front();
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop_front();
				if (!frontier.empty())
					current_node = frontier.front();
			}
		}
	}

	if (SolutionFound)
	{
		plan = current_node.secuencia;
	}

	return plan;
}

list<Action> ComportamientoJugador::algoritmoDijkstra(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{

	nodeN02 current_node;
	priority_queue<nodeN02, vector<nodeN02>, comparacion> frontier;
	list<Action> plan;
	plan = current_node.secuencia;
	set<nodeN02> explored;

	current_node.st = inicio;
	current_node.g = 0;

	bool SolutionFound = (current_node.st.jugador.f == final.f && current_node.st.jugador.c == final.c);
	frontier.push(current_node);

	while (!frontier.empty() && !SolutionFound)
	{
		frontier.pop();

		if (mapa[current_node.st.jugador.f][current_node.st.jugador.c] == 'K')
		{
			current_node.st.bikini_j = true;
			current_node.st.zapas_j = false;
		}
		else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c] == 'D')
		{
			current_node.st.zapas_j = true;
			current_node.st.bikini_j = false;
		}

		explored.insert(current_node);

		// Generar hijo actrFORWRAD
		nodeN02 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (child_forward.st.jugador.f == final.f && child_forward.st.jugador.c == final.c)
		{
			child_forward.secuencia.push_back(actFORWARD);
			child_forward.g += coste(actFORWARD, child_forward.st, mapa);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actFORWARD);
			child_forward.g += coste(actFORWARD, child_forward.st, mapa);
			frontier.push(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hojo actTRURN_L
			nodeN02 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				child_turnl.g += coste(actTURN_L, child_turnl.st, mapa);
				frontier.push(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN02 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				child_turnr.g += coste(actTURN_R, child_turnr.st, mapa);
				frontier.push(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
		{
			current_node = frontier.top();
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop();
				if (!frontier.empty())
					current_node = frontier.top();
			}
		}
	}

	if (SolutionFound)
	{

		plan = current_node.secuencia;
	}

	return plan;
}

/*	NIVEL 3	*/

list<Action> ComportamientoJugador::algoritmoNivel3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{

	nodeN02 current_node;
	priority_queue<nodeN02, vector<nodeN02>, comparacion> frontier;
	list<Action> plan;
	plan = current_node.secuencia;
	set<nodeN02> explored;

	current_node.st = inicio;
	current_node.g = 0;
	current_node.h = distancia_manhattan(current_node.st.sonambulo, final);

	bool SolutionFound = (current_node.st.sonambulo.f == final.f && current_node.st.sonambulo.c == final.c);
	frontier.push(current_node);

	list<Action> jugador;
	jugador = jugadorNivel3(current_node.st, current_node.st.sonambulo, mapa);

	for (auto it = jugador.begin(); it != jugador.end(); ++it)
	{
		current_node.secuencia.push_back(*it);
		current_node.st = apply(*it, current_node.st, mapa);
	}

	list<Action> sonam;
	sonam = sonambuloNivel3(current_node.st, final, mapa);

	while (!sonam.empty())
	{
		current_node.secuencia.push_back(sonam.front());
		current_node.st = apply(sonam.front(), current_node.st, mapa);
		sonam.pop_front();

		if (!ve_sonambulo(current_node.st, mapa) && !sonam.empty())
		{
			list<Action> jugador2;
			jugador2 = jugadorNivel3(current_node.st, current_node.st.sonambulo, mapa);

			for (auto it = jugador2.begin(); it != jugador2.end(); ++it)
			{
				current_node.secuencia.push_back(*it);
				current_node.st = apply(*it, current_node.st, mapa);
			}
		}
	}

	plan = current_node.secuencia;

	return plan;
}

int ComportamientoJugador::coste(Action accion, const stateN0 &st, const vector<vector<unsigned char>> &mapa)
{
	int coste = 0;
	switch (mapa[st.jugador.f][st.jugador.c])
	{
	case 'A':

		switch (accion)
		{
		case actFORWARD:
			if (st.bikini_j)
			{
				coste = 10;
				break;
			}
			else
			{
				coste = 100;
				break;
			}
		case actSON_FORWARD:
			if (st.bikini_s)
			{
				coste = 10;
				break;
			}
			else
			{
				coste = 10;
				break;
			}
		case actTURN_L:
			if (st.bikini_j)
			{
				coste = 5;
				break;
			}
			else
			{
				coste = 25;
				break;
			}
		case actTURN_R:
			if (st.bikini_j)
			{
				coste = 5;
				break;
			}
			else
			{
				coste = 25;
				break;
			}
		case actSON_TURN_SL:
			if (st.bikini_s)
			{
				coste = 2;
				break;
			}
			else
			{
				coste = 7;
				break;
			}
		case actSON_TURN_SR:
			if (st.bikini_s)
			{
				coste = 2;
				break;
			}
			else
			{
				coste = 7;
				break;
			}
		}
		break;
	case 'B':
		switch (accion)
		{
		case actFORWARD:
			if (st.zapas_j)
			{
				coste = 15;
				break;
			}
			else
			{
				coste = 50;
				break;
			}
		case actSON_FORWARD:
			if (st.zapas_s)
			{
				coste = 15;
				break;
			}
			else
			{
				coste = 50;
				break;
			}
		case actTURN_L:
			if (st.zapas_j)
			{
				coste = 1;
				break;
			}
			else
			{
				coste = 5;
				break;
			}
		case actTURN_R:
			if (st.zapas_j)
			{
				coste = 1;
				break;
			}
			else
			{
				coste = 5;
				break;
			}
		case actSON_TURN_SL:
			if (st.zapas_s)
			{
				coste = 1;
				break;
			}
			else
			{
				coste = 3;
				break;
			}
		case actSON_TURN_SR:
			if (st.zapas_s)
			{
				coste = 1;
				break;
			}
			else
			{
				coste = 3;
				break;
			}
		}
		break;
	case 'T':
		switch (accion)
		{
		case actFORWARD:
			coste = 2;
			break;
		case actSON_FORWARD:
			coste = 2;
			break;
		case actTURN_L:
			coste = 2;
			break;
		case actTURN_R:
			coste = 2;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'S':
		switch (accion)
		{
		case actFORWARD:
			coste = 1;
			break;
		case actSON_FORWARD:
			coste = 1;
			break;
		case actTURN_L:
			coste = 1;
			break;
		case actTURN_R:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'K':
		switch (accion)
		{
		case actFORWARD:
			coste = 1;
			break;
		case actSON_FORWARD:
			coste = 1;
			break;
		case actTURN_L:
			coste = 1;
			break;
		case actTURN_R:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'D':
		switch (accion)
		{
		case actFORWARD:
			coste = 1;
			break;
		case actSON_FORWARD:
			coste = 1;
			break;
		case actTURN_L:
			coste = 1;
			break;
		case actTURN_R:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'X':
		switch (accion)
		{
		case actFORWARD:
			coste = 1;
			break;
		case actSON_FORWARD:
			coste = 1;
			break;
		case actTURN_L:
			coste = 1;
			break;
		case actTURN_R:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	}

	return coste;
}

int ComportamientoJugador::distancia_manhattan(const ubicacion ubi, const ubicacion &final)
{
	int distancia;
	distancia = abs(ubi.f - final.f) + abs(ubi.c - final.c);
	return distancia;
}

list<Action> ComportamientoJugador::sonambuloNivel3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{
	nodeN02 current_node;
	priority_queue<nodeN02, vector<nodeN02>, comparacion> frontier;
	list<Action> plan;
	plan = current_node.secuencia;
	set<nodeN02> explored;

	current_node.st = inicio;
	current_node.g = 0;
	current_node.h = distancia_manhattan(current_node.st.sonambulo, final);

	bool SolutionFound = (current_node.st.sonambulo.f == final.f && current_node.st.sonambulo.c == final.c);
	frontier.push(current_node);

	while (!frontier.empty() && !SolutionFound)
	{
		frontier.pop();

		if (mapa[current_node.st.sonambulo.f][current_node.st.sonambulo.c] == 'K')
		{
			current_node.st.bikini_s = true;
			current_node.st.zapas_s = false;
		}
		else if (mapa[current_node.st.sonambulo.f][current_node.st.sonambulo.c] == 'D')
		{
			current_node.st.zapas_s = true;
			current_node.st.bikini_s = false;
		}

		explored.insert(current_node);

		// Generar hijo actrFORWRAD
		nodeN02 child_forward = current_node;
		child_forward.st = apply(actSON_FORWARD, current_node.st, mapa);
		if (child_forward.st.sonambulo.f == final.f && child_forward.st.sonambulo.c == final.c)
		{
			child_forward.secuencia.push_back(actSON_FORWARD);
			child_forward.g += coste_s(actSON_FORWARD, child_forward.st, mapa);
			child_forward.h = distancia_manhattan(child_forward.st.sonambulo, final);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actSON_FORWARD);
			child_forward.g += coste_s(actSON_FORWARD, child_forward.st, mapa);
			child_forward.h = distancia_manhattan(child_forward.st.sonambulo, final);
			frontier.push(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hojo actTRURN_L
			nodeN02 child_turnl = current_node;
			child_turnl.st = apply(actSON_TURN_SL, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actSON_TURN_SL);
				child_turnl.g += coste_s(actSON_TURN_SL, child_turnl.st, mapa);
				child_turnl.h = distancia_manhattan(child_turnl.st.sonambulo, final);
				frontier.push(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN02 child_turnr = current_node;
			child_turnr.st = apply(actSON_TURN_SR, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actSON_TURN_SR);
				child_turnr.g += coste_s(actSON_TURN_SR, child_turnr.st, mapa);
				child_turnr.h = distancia_manhattan(child_turnr.st.sonambulo, final);
				frontier.push(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
		{
			current_node = frontier.top();
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop();
				if (!frontier.empty())
					current_node = frontier.top();
			}
		}
	}

	if (SolutionFound)
	{
		plan = current_node.secuencia;
	}

	return plan;
}

list<Action> ComportamientoJugador::jugadorNivel3(const stateN0 &inicio, const ubicacion &final, const vector<vector<unsigned char>> &mapa)
{

	nodeN02 current_node;
	priority_queue<nodeN02, vector<nodeN02>, comparacion> frontier;
	list<Action> plan;
	plan = current_node.secuencia;
	set<nodeN02> explored;

	current_node.st = inicio;
	current_node.g = 0;
	current_node.h = distancia_manhattan(current_node.st.jugador, current_node.st.sonambulo);

	bool SolutionFound = (ve_sonambulo(current_node.st, mapa));
	frontier.push(current_node);

	while (!frontier.empty() && !SolutionFound)
	{
		frontier.pop();

		if (mapa[current_node.st.jugador.f][current_node.st.jugador.c] == 'K')
		{
			current_node.st.bikini_j = true;
			current_node.st.zapas_j = false;
		}
		else if (mapa[current_node.st.jugador.f][current_node.st.jugador.c] == 'D')
		{
			current_node.st.zapas_j = true;
			current_node.st.bikini_j = false;
		}

		explored.insert(current_node);

		// Generar hijo actrFORWRAD
		nodeN02 child_forward = current_node;
		child_forward.st = apply(actFORWARD, current_node.st, mapa);
		if (ve_sonambulo(child_forward.st, mapa))
		{
			child_forward.secuencia.push_back(actFORWARD);
			child_forward.g += coste(actFORWARD, child_forward.st, mapa);
			child_forward.h = distancia_manhattan(child_forward.st.jugador, child_forward.st.sonambulo);
			current_node = child_forward;
			SolutionFound = true;
		}
		else if (explored.find(child_forward) == explored.end())
		{
			child_forward.secuencia.push_back(actFORWARD);
			child_forward.g += coste(actFORWARD, child_forward.st, mapa);
			child_forward.h = distancia_manhattan(child_forward.st.jugador, child_forward.st.sonambulo);
			frontier.push(child_forward);
		}

		if (!SolutionFound)
		{
			// Generar hojo actTRURN_L
			nodeN02 child_turnl = current_node;
			child_turnl.st = apply(actTURN_L, current_node.st, mapa);
			if (explored.find(child_turnl) == explored.end())
			{
				child_turnl.secuencia.push_back(actTURN_L);
				child_turnl.g += coste(actTURN_L, child_turnl.st, mapa);
				child_turnl.h = distancia_manhattan(child_turnl.st.jugador, child_turnl.st.sonambulo);
				frontier.push(child_turnl);
			}

			// Generar hijo actTURN_R
			nodeN02 child_turnr = current_node;
			child_turnr.st = apply(actTURN_R, current_node.st, mapa);
			if (explored.find(child_turnr) == explored.end())
			{
				child_turnr.secuencia.push_back(actTURN_R);
				child_turnr.g += coste(actTURN_R, child_turnr.st, mapa);
				child_turnr.h = distancia_manhattan(child_turnr.st.jugador, child_turnr.st.sonambulo);
				frontier.push(child_turnr);
			}
		}

		if (!SolutionFound && !frontier.empty())
		{
			current_node = frontier.top();
			while (!frontier.empty() && explored.find(current_node) != explored.end())
			{
				frontier.pop();
				if (!frontier.empty())
					current_node = frontier.top();
			}
		}
	}

	if (SolutionFound)
	{

		plan = current_node.secuencia;
	}

	return plan;
}

int ComportamientoJugador::coste_s(Action accion, const stateN0 &st, const vector<vector<unsigned char>> &mapa){
	int coste = 0;
	switch (mapa[st.sonambulo.f][st.sonambulo.c])
	{
	case 'A':

		switch (accion)
		{
		case actSON_FORWARD:
			if (st.bikini_s)
			{
				coste = 10;
				break;
			}
			else
			{
				coste = 10;
				break;
			}
		case actSON_TURN_SL:
			if (st.bikini_s)
			{
				coste = 2;
				break;
			}
			else
			{
				coste = 7;
				break;
			}
		case actSON_TURN_SR:
			if (st.bikini_s)
			{
				coste = 2;
				break;
			}
			else
			{
				coste = 7;
				break;
			}
		}
		break;
	case 'B':
		switch (accion)
		{
		case actSON_FORWARD:
			if (st.zapas_s)
			{
				coste = 15;
				break;
			}
			else
			{
				coste = 50;
				break;
			}
		case actSON_TURN_SL:
			if (st.zapas_s)
			{
				coste = 1;
				break;
			}
			else
			{
				coste = 3;
				break;
			}
		case actSON_TURN_SR:
			if (st.zapas_s)
			{
				coste = 1;
				break;
			}
			else
			{
				coste = 3;
				break;
			}
		}
		break;
	case 'T':
		switch (accion)
		{
		case actSON_FORWARD:
			coste = 2;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'S':
		switch (accion)
		{
		case actSON_FORWARD:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'K':
		switch (accion)
		{
		case actSON_FORWARD:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'D':
		switch (accion)
		{
		case actSON_FORWARD:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	case 'X':
		switch (accion)
		{
		case actSON_FORWARD:
			coste = 1;
			break;
		case actSON_TURN_SL:
			coste = 1;
			break;
		case actSON_TURN_SR:
			coste = 1;
			break;
		}
		break;
	}

	return coste;
}