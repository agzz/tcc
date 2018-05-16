/*
 * main.cpp
 *
 * Copyright (c) 2018 Adriano Zenzen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/**
 * TODO Input
 * TODO EXTI
 * TODO Endstops
 * TODO Homing
 * TODO Considerar movimentação por lista e interrupção
 */

#include "stm32f4xx_hal.h"

#include <string>

#include "DigitalOut.h"
#include "Serial.h"
#include "Stepper.h"
#include "clock.h"

//#define PROTOTIPO

//Limite de movimentação dos eixos em graus
#define X_MAX 360.0
#define Y_MAX 360.0

//Passos necessários por grau
#ifndef PROTOTIPO
#define STEPS_DEGREE (20000/360)
#else
#define STEPS_DEGREE (2048.0/360)
#endif

//Velocidades máxima e mínima de movimentação em graus/min
#define MAX_FEEDRATE (54*60)
#define MIN_FEEDRATE (9*60)

//Posição atual do sistema
float xPos = 0.0;
float yPos = 0.0;

//Velocidade de movimentação do sistema
int feedrate = 18*60;
uint32_t stepDelay = ((1000000*60/feedrate)/STEPS_DEGREE)-2;

//Se true, modo absoluto de movimentação, se false, modo relativo
bool absoluteMode = true;

//Comunicação serial
Serial* serial;

//Motores
#ifndef PROTOTIPO
Stepper xAxis(PA8, PB10, PB4, false);
Stepper yAxis(PB5, PB3, PA10, false);
#else
Stepper xAxis(PA10, PB4, false);
Stepper yAxis(PB3, PB10, false);
DigitalOut* enable;
#endif

/**
 * Recebe uma string e analisa ela em busca de comandos
 *
 */
void parseCommand(std::string command);

/**
 * Recebe uma string e um char, procura pelo char na string
 * e retorna, caso haja, os números seguidos do char como um
 * inteiro
 *
 * str				string em que será feita a busca
 * key				char a ser buscado na string
 * notFound			retorno caso haja algum problema
 *
 */
int parseInt(std::string str, char key, int notFound);

/**
 * Recebe uma string e um char, procura pelo char na string
 * e retorna, caso haja, os números seguidos do char como um
 * float
 *
 * str				string em que será feita a busca
 * key				char a ser buscado na string
 * notFound			retorno caso haja algum problema
 *
 */
float parseFloat(std::string str, char key, float notFound);

/**
 * Faz a movimentação em linha de um ponto a outro
 *
 * newx				coordenada x do final do movimento
 * newy				coordenada y do final do movimento
 *
 */
void line(float newx, float newy);

int main(void) {

	//Configurações iniciais
	HAL_Init();
	clockConfig();

	//Inicialização do led
	DigitalOut led(PA5);
	if (led.getError()) {
		while(1);
	}

#ifdef PROTOTIPO
	enable = new DigitalOut(PA9);
	if (enable->getError()) {
		while(1);
	}
	enable->set();
#endif

	//Inicialização da Serial
	serial = new Serial(USART2, 115200, 128);
	if (serial->getError()) {
		while(1);
	}

	//String para armazenar o comando recebido pela serial
	std::string command;

	while (1) {
		//Checa por dados na serial
		if (serial->available()) {
			uint8_t value;

			//Recebe um caractere
			serial->getChar(&value);

			//Verifica se é algum caractere de final de comando
			if (value == '\n' || value == '\r') {
				//Se houver dados na string analisa o comando (checa por sequências \n\r e \r\n)
				if (!command.empty()) {
					//Imprime o comando
					serial->println(command);

					//Analisa e executa o comando
					parseCommand(command);
					command.clear();
				}

				led.toggle();
			} else {
				//Se não for caractere de final de comando
				//Limita o tamanho da string em 64 bytes
				if (command.size() >= 64) {
					command.erase(0);
				}

				//Adiciona o caractere na string
				command.push_back((char) value);
			}
		}

		//led.toggle();
		//usDelay(10);
	}
}

/**
 * Recebe uma string e analisa ela em busca de comandos
 *
 */
void parseCommand(std::string command) {
	int cmd;

	//Checar se é um comando G
	cmd = parseInt(command, 'G', -1);
	if (cmd != -1) {
		switch(cmd) {
		case 0:
		case 1:
			//Mover em linha
			//Obter o valor da velocidade, manter o mesmo caso não haja
			feedrate = parseInt(command, 'F', feedrate);

			if (feedrate >= MAX_FEEDRATE) {
				feedrate = MAX_FEEDRATE;
			} else if (feedrate <= MIN_FEEDRATE) {
				feedrate = MIN_FEEDRATE;
			}

			stepDelay = ((1000000*60/feedrate)/STEPS_DEGREE)-2;

			//Obter os valores de X e Y e fazer a movimentação
			if (absoluteMode) {
				line(parseFloat(command, 'X', xPos), parseFloat(command, 'Y', yPos));
			} else {
				line(xPos+parseFloat(command, 'X', 0), yPos+parseFloat(command, 'Y', 0));
			}
			break;

		case 4:
			//Esperar
			HAL_Delay(parseInt(command, 'P', 0)*1000);
			break;

		case 90:
			//Mudar para modo absoluto
			absoluteMode = true;
			break;

		case 91:
			//Mudar para modo relativo
			absoluteMode = false;
			break;

		case 92:
			//Setar a posição atual
			xPos = parseFloat(command, 'X', xPos);
			yPos = parseFloat(command, 'Y', yPos);
			break;

		default:
			break;
		}
	}

	//Checar se é um comando M
	cmd = parseInt(command, 'M', -1);
	if (cmd != -1) {
		switch(cmd) {
		case 17:
			//Habilitar motores
#ifndef PROTOTIPO
			xAxis.enable();
			yAxis.enable();
#else
			enable->reset();
#endif
			break;

		case 18:
			//Desabilitar motores
#ifndef PROTOTIPO
			xAxis.disable();
			yAxis.disable();
#else
			enable->set();
#endif
			break;

		case 100:
			//Imprimir lista de comandos
			//help();
			break;

		case 114:
			//Dizer a posição atual e velocidade
			serial->println("X:%.3f, Y:%.3f, F:%d", xPos, yPos, feedrate);
			break;

		default:
			break;
		}
	}
}

/**
 * Recebe uma string e um char, procura pelo char na string
 * e retorna, caso haja, os números seguidos do char como um
 * inteiro
 *
 * str				string em que será feita a busca
 * key				char a ser buscado na string
 * notFound			retorno caso haja algum problema
 *
 */
int parseInt(std::string str, char key, int notFound) {
	//Procurar pelo caractere e achar a posição na string
	size_t pos = str.find(key);
	if (pos != std::string::npos) {
		//Obter a substring a partir do caractere
		std::string substring = str.substr(pos+1);

		//Checar se o primeiro caractere é um número
		if (!isdigit(substring.c_str()[0])) {
			//Não é um número, checar se é um sinal negativo e o seguinte um número
			if ( !( substring.c_str()[0] == '-' && isdigit(substring.c_str()[1])) ) {
				//Também não, retornar o valor de não encontrado.
				return notFound;
			}
		}

		//É um número, converter e retornar o valor
		return atoi(substring.c_str());
	}

	//Caractere não encontrado, retornar o valor de não encontrado
	return notFound;
}

/**
 * Recebe uma string e um char, procura pelo char na string
 * e retorna, caso haja, os números seguidos do char como um
 * float
 *
 * str				string em que será feita a busca
 * key				char a ser buscado na string
 * notFound			retorno caso haja algum problema
 *
 */
float parseFloat(std::string str, char key, float notFound) {
	//Procurar pelo caractere e achar a posição na string
	size_t pos = str.find(key);
	if (pos != std::string::npos) {
		//Obter a substring a partir do caractere
		std::string substring = str.substr(pos+1);

		//Checar se o primeiro caractere é um número
		if (!isdigit(substring.c_str()[0])) {
			//Não é um número, checar se é um sinal negativo e o seguinte um número
			if ( !( substring.c_str()[0] == '-' && isdigit(substring.c_str()[1])) ) {
				//Também não, retornar o valor de não encontrado.
				return notFound;
			}
		}

		//É um número, converter e retornar o valor
		return atof(substring.c_str());
	}

	//Caractere não encontrado, retornar o valor de não encontrado
	return notFound;
}

/**
 * Faz a movimentação em linha de um ponto a outro
 *
 * newx				coordenada x do final do movimento
 * newy				coordenada y do final do movimento
 *
 */
void line(float newx,float newy) {
    int32_t over = 0;

    //Garantir limites do eixo X
    if (newx >= X_MAX) {
        newx = X_MAX;
    } else if (newx <= 0) {
    	newx = 0;
    }

    //Garantir limites do eixo Y
    if (newy >= Y_MAX) {
        newy = Y_MAX;
    } else if (newy <= 0) {
    	newy = 0;
    }

    //Calcular quanto mover cada eixo
    int32_t dx  = (newx-xPos)*STEPS_DEGREE;
    int32_t dy  = (newy-yPos)*STEPS_DEGREE;

    //Checar a direção de movimento
    if (dx > 0) {
        xAxis.direction(CW);
    } else {
        xAxis.direction(CCW);
    }

    if (dy > 0) {
        yAxis.direction(CW);
    } else {
        yAxis.direction(CCW);
    }

    dx = abs(dx);
    dy = abs(dy);

    //Movimentar
    if (dx > dy) {
        over = dx/2;
        for (int32_t i = 0; i < dx; ++i) {
            xAxis.step();
            over += dy;

            if (over >= dx) {
                over -= dx;
                yAxis.step();
            }

            usDelay(stepDelay);
        }
    } else {
        over = dy/2;
        for (int32_t i = 0; i < dy; ++i) {
            yAxis.step();
            over += dx;

            if (over >= dy) {
                over -= dy;
                xAxis.step();
            }

            usDelay(stepDelay);
        }
    }

    //Atualizar as posições
    xPos = newx;
    yPos = newy;
}

