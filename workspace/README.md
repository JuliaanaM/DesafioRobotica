# Desafio de Robótica

   ###### ![image](https://user-images.githubusercontent.com/21108858/110886659-c62ef100-82c7-11eb-9f5b-b1bb849f3a5e.png)
                            
Este repositório contém os arquivos necessários para a realização da simulação do desafio referente ao Laboratório de Robótica e Sistemas Autônomos para atuação como estagiário ou bolsista no SENAI CIMATEC.

# Objetivo
O robô deverá chegar na região de uma luminária de chão que está ao lado de uma placa STOP em no máximo 2 minutos, partindo do ponto inicial próximo a placa INÍCIO.
# Resolução
As funções implementadas estão apresentadas abaixo:

### ![image](https://user-images.githubusercontent.com/21108858/110886799-fb3b4380-82c7-11eb-9172-e573956c4599.png)
                                            
_**Init System**_: Inicializa a comunicação entre o controlador e o Webots e armazena o tempo de simulação e os IDs das rodas, leds e sensores. Configura as rodas e habilita os sensores. Configura o estado forward (em frente) como default da máquina de estados da direção do robô.

_**Avoid Obstacle**_: Verifica se há obstáculos próximos através da distância calculada a partir dos valores obtidos pelos sensores. Se houver um obstáculo, é calculado um modificador que ajusta os parâmetros das rodas, através do qual é atualizada a direção do robô na máquina de estados. 

_**Stop Position**_: O robô identifica o local em que ele deve parar através da análise da intensidade de luminosidade obtidas pelos sensores de luminosidade. Nesta função é obtida a média dos valores de saída dos sensores.

_**Direction State Machine**_: Gerencia a direção do robô: forward (em frente), left (esquerda) e right (direita).

_**Set Motor Speed**_: Ajusta a velocidade das rodas possibilitando que o robô modifique sua direção ao encontrar um obstáculo ou pare ao identificar o ponto de parada.

_**Robot Cleanup**_: Encerra a comunicação entre o controlador e o Webots.

## Principais ajustes
Para que o robô se tornasse capaz de cumprir a missão foi feita uma correção no cálculo da distância, corrigindo o valor do parâmetro da distância máxima (Max_distance) na função:
###### _distance = Max_distance * (1.0 - (sensor_value / MAX_SENSOR_VALUE))_
                   
A distância máxima de detecção dos sensores do Pioneer 3-DX , conforme indicado na lookup table, é igual 5m.

Porm fim, foram adicionados três sensores de luz para a identificação do ponto de parada através da intensidade da luminosidade do local.
# Resultado
O robô é capaz de chegar ao ponto de parada, evitando os obstáculos, em um tempo de execução de 1 min 16 s.
# Referência 
1. Cyberbotics. **Lightsensor.** Acesso em: 8 de Março de 2021 [https://cyberbotics.com/doc/reference/lightsensor].
2. Cyberbotics. **Pioneer 3-DX.** Acesso em: 8 de Março de 2021 [https://cyberbotics.com/doc/guide/pioneer-3dx].
3. Cyberbotics. **Distance Sensor.** Acesso em: 8 de Março de 2021 [https://cyberbotics.com/doc/reference/distancesensor].


