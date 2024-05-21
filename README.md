# Controlador de Robô

Este repositório contém o código para um sistema de controle de robôs, implementado em Python com ROS 2. Ele permite que o usuário controle um robô de forma interativa através de uma interface de linha de comando.

## Descrição

O sistema de controle do robô permite operações como conectar, desconectar, mover manualmente o robô, parar e executar uma parada de emergência. Ele é construído utilizando a biblioteca rclpy para interação com o ROS 2 e inquirer para a interface de usuário na linha de comando.

### Funcionalidades

- **Conectar**: Estabelece uma conexão com o robô, permitindo comandos subsequentes.
- **Desconectar**: Encerra a conexão com o robô.
- **Mover**: Permite o controle manual do robô usando as teclas 'w', 's', 'a', 'd'. Além, das teclas "q" para retornar e "b" para forçar parada.
- **Parada de Emergência**: Executa uma parada de emergência que também notifica o sistema ROS sobre a necessidade de parada imediata, assim cancelando a execução do código de bringup no robô, cancelando completamente qualquer interação da CLI com o robô.

## Como Funciona

O sistema utiliza o ROS 2 para a comunicação com o robô e implementa uma interface de linha de comando para interação com o usuário. O código é estruturado em torno de uma classe RobotController, que gerencia a conexão com o robô, envio de comandos e execução de paradas de emergência.

## Pré-requisitos

- Python 3.8 ou superior
- ROS 2
- Bibliotecas Python: typer, inquirer

## Configuração e Instalação

Primeiro, atualize o seus sistema, digitando o seguinte no terminal

```bash
sudo apt update

sudo apt upgrade
```

Certifique-se de que o ROS 2 está corretamente instalado em seu sistema.


Digite ROS2 em seu terminal, se retornar comando não existente, siga este tutorial para instalar o [ROS2](https://docs.ros.org/en/foxy/Installation.html)
```bash
ros2
``` 

Além disto, certifique-se de ter tanto python3 quanto VENV baixados. Para checar digite o seguinte no seu terminal

```bash
sudo apt install python3

sudo apt-get install python3-venv
``` 


## Como Executar

Para iniciar tanto a CLI quanto o robô para escutar os comandos enviados, digite o seguinte em seu terminal:

Primeiro, se conecte via SSH com o robô, para tal feito digite o seguinte no terminal, vale ressaltar que o comando do SSH irá variar dependendo do IP da Raspberry Pi de seu robô

```bash
ssh grupo4@10.128.0.9
```

Após se conectar ao robô, execute o código de bringup, levemente modificado afim de enquadrar o sistema de emergência.

1. Entre no diretório do workspace do bringup

```bash
cd new_bringup_ws
```

2. Rode o comando colcon build para buildar os arquivos

```bash
colcon build
```

3. De source no código

```bash
source install/local_setup.bash
```

4. Rode o código do novo bringup

```bash
ros2 run new_bringup bringup_manager
```

Após isto, seu robô estará pronto para receber informações via ROS.

### Execução da CLI

Para executar a CLI, deve-se inicialmente entrar no diretório deste github.

```bash
cd Ponderada_Turtlebot_Pt.1
```

Após, rode o script para inicializar o ROS, o script a seguir além de instalar o VENV, as depências do python3, ele também ja builda e da source no código do ROS2, além de rodar o mesmo

```bash
./exec.sh
```

Se der algum erro, execute o seguinte

```bash
chmod +x ./exec.sh
```

E execute novamente

```bash
./exec.sh
```

Após isto, a CLI está pronta para ser utilizada.

OBS: (A movimentação do robô só funciona se o mesmo está na mesma rede Wi-Fi que a maquina rodando a CLI)

## Licença

Este projeto é distribuído sob a licença MIT. Veja o arquivo LICENSE para mais detalhes.

## Contato

Se você tiver alguma dúvida ou sugestão, não hesite em entrar em contato através do e-mail fornecido na configuração do pacote.

---

Este README fornece uma visão geral básica do projeto e instruções para que usuários possam começar a usar e contribuir para o desenvolvimento do sistema de controle do robô.
