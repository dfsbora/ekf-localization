<h1 align="center">Localização EKF para robô NAO</h1>
<p align="center">Implementação de sistema de localização para futebol de robôs baseado em unidade inercial</p>


<p align="center">
  <img alt="GitHub" src="https://img.shields.io/github/license/dfsbora/ekf-localization">
</p>

[English version](README.md)

## Requisitos
Desenvolvido em Python 2.7

🤖 O robô NAO deve cumprir os seguintes requisitos:

* NAOqi (Verificado com 2.8.5.10)

* [Numpy](https://numpy.org/) (Verificado com 1.10.4)

* [Scipy](https://www.scipy.org/) (Verificado com v1.2.3)



## Executando o cdigo

#### Copiando o código par ao robô

```bash
# Torna o script executável
$ chmod +x send.sh

# Envia arquivos para o robô
$ ./send.sh <robot-ip>

```

#### Executando o código no robô

```bash
# Conecta ao robô NAO
$ ssh nao@<robot-ip>

# Acessa a pasta do projeto
$ cd naoqi/ekf-localization

# Executa o cdigo
$ python threads.py

```


## Licença

This project is under the license [MIT](./LICENSE).

Feito por Débora Ferreira. 🤖💚
