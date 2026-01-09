# LABSIS AGV 25/26 — Seguidor de Linha (ATmega88)

Projeto desenvolvido no âmbito da unidade curricular **LABSIS (2025/2026)**.  
Consiste num **AGV seguidor de linha** baseado no microcontrolador **ATmega88**, com leitura de sensores óticos, controlo de motores DC via ponte H, e medição de velocidade por encoders.

## Conteúdo do repositório
- `main.c` — firmware em C (AVR) do AGV
- `docs/` — relatório em formato Web (HTML) com imagens, tabelas e fluxogramas

## Ver o relatório (GitHub Pages)
Se o GitHub Pages estiver ativo, o relatório pode ser aberto diretamente no browser através do link do Pages do repositório.

Caso não esteja ativo:
1. Abrir a pasta `docs/`
2. Fazer download do repositório
3. Abrir o ficheiro `docs/index.html` num browser

## Resumo do funcionamento
O controlo do robot é organizado em duas camadas:
1. **Seguimento de linha (PD):** calcula o erro a partir dos sensores e ajusta as referências de velocidade das rodas para recentrar o AGV na faixa.
2. **Controlo de velocidade (PI):** mede a velocidade real com encoders e ajusta o PWM para atingir as referências, compensando variações de carga/bateria.

## Autores
- João Pedro Ribeiro Bastos 
- Pedro Coelho
