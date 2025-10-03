# Parâmetros do EKF Localizer

Este documento descreve os novos parâmetros configuráveis do filtro de Kalman estendido (EKF) no localizador.

## Arquivos Modificados

- `localizer_node.cpp`: Adicionada função `loadEKFParams()` para carregar parâmetros via ROSParam
- `localizer_node.h`: Adicionada declaração da função `loadEKFParams()`
- `ekf_params.yaml`: Arquivo de configuração com os parâmetros do EKF
- `run_ekf_localizer.launch`: Atualizado para carregar os parâmetros do EKF

## Parâmetros Configuráveis

### Matriz P Inicial (Covariância do Estado Inicial)

- `ekf_params/initial_covariance/position_x`: Covariância inicial da posição X (padrão: 0.5 m²)
- `ekf_params/initial_covariance/position_y`: Covariância inicial da posição Y (padrão: 0.5 m²)
- `ekf_params/initial_covariance/orientation`: Covariância inicial da orientação (padrão: 0.5 rad²)

### Matriz Q (Covariância do Processo)

- `ekf_params/process_covariance/position_x`: Covariância do processo para posição X (padrão: 0.0005 m²)
- `ekf_params/process_covariance/position_y`: Covariância do processo para posição Y (padrão: 0.0005 m²)

## Como Usar

1. **Via arquivo de configuração**: Edite o arquivo `ekf_params.yaml` com os valores desejados
2. **Via parâmetros ROS**: Use `rosparam set` para definir valores em tempo de execução
3. **Via linha de comando**: Passe os parâmetros diretamente no launch file

### Exemplos de Uso

```bash
# Definir parâmetros via rosparam
rosparam set /ekf_params/initial_covariance/position_x 1.0
rosparam set /ekf_params/process_covariance/position_x 0.001

# Ou via linha de comando no launch
roslaunch conf run_ekf_localizer.launch ekf_params/initial_covariance/position_x:=1.0
```

## Valores Padrão

Se os parâmetros não forem fornecidos, o sistema usará os valores padrão originais:
- P inicial: [0.5, 0.5, 0.5] (posição x, posição y, orientação)
- Q: [0.0005, 0.0005] (posição x, posição y)

## Logs

O sistema irá imprimir os parâmetros carregados no log de inicialização:
```
[LocalizerNode] EKF Parameters loaded:
  - Initial covariance P: [0.500000, 0.500000, 0.500000]
  - Process covariance Q: [0.000500, 0.000500]
```
