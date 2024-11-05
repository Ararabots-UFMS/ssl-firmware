# Código que calcula o valor de Counter Period para o PWM do microcontrolador STM32F103C8T6
# dado um valor de frequência desejada e um valor de prescaler a ser utilizado.

freq_desejada = float(input("Digite a frequência desejada: "))

period_desejado = 1 / freq_desejada

prescaler = int(input("Digite o valor do prescaler: "))

freq = 64000000 / prescaler

period = 1 / freq

counter_period = period_desejado / period

print("Counter Period: ", round(counter_period))
