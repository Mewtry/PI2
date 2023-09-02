import os
import subprocess
import serial.tools.list_ports

def escolher_porta_usb():
    # Busca portas COM disponíveis
    portas_disponiveis = serial.tools.list_ports.comports(True)

    # Exibir as portas USB disponíveis para o usuário escolher
    print("Portas USB disponíveis:")
    for i, porta in enumerate(portas_disponiveis):
        print(f"{i + 1}. {porta}")

    # Pede interação do usuário para escolha da porta
    escolha = int(input("Digite o número da porta USB que deseja utilizar: "))

    # Verificar se a escolha é válida
    while escolha < 1 or escolha > len(portas_disponiveis):
        print("Escolha inválida. Tente novamente.")
        escolha = int(input("Digite o número da porta USB que deseja utilizar: "))

    return portas_disponiveis[escolha - 1].device
    

def executar_arquivo_usb(usb):
    # chose an implementation, depending on os
    if os.name == 'nt':  # sys.platform == 'win32':
        comando = os.path.expanduser('~') + r'\\AppData\\Local\\Arduino15\\packages\\esp32\\tools\\esptool_py\\4.5.1/esptool.exe --chip esp32 --port ' + usb + r' --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 "build/MYT_600.ino.bootloader.bin" 0x8000 "build/MYT_600.ino.partitions.bin" 0xe000 "' + os.path.expanduser('~') + r'\\AppData\\Local\\Arduino15\\packages\\esp32\\hardware\\esp32\\2.0.9/tools/partitions/boot_app0.bin" 0x10000 "build/MYT_600.ino.bin"'
    elif os.name == 'posix': # sys.plataform == 'Darwin' (macOs)
        comando = os.path.expanduser('~') + r'/Library/Arduino15/packages/esp32/tools/esptool_py/4.5.1/esptool --chip esp32 --port ' + usb + r' --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 "build/MYT_600.ino.bootloader.bin" 0x8000 "build/MYT_600.ino.partitions.bin" 0xe000 "' + os.path.expanduser('~') + r'/Library/Arduino15/packages/esp32/hardware/esp32/2.0.9/tools/partitions/boot_app0.bin" 0x10000 "build/MYT_600.ino.bin"'
    try:
        subprocess.call(comando, shell=True)
    except Exception as e:
        print(f"Erro ao executar o arquivo: {e}")

# Listar as portas USB disponíveis e pedir para o usuário escolher qual usar
porta_usb = escolher_porta_usb()
# Executar o arquivo na porta USB selecionada
executar_arquivo_usb(porta_usb)

while True:
    # Apresentar opções ao usuário
    print("===============================================================================================")
    print("Aperte ENTER para regravar.")
    print('Aperte a tecla "1" para alterar a porta COM.')
    print('Aperte a tecla "0" para finalizar o processo.')
    print("===============================================================================================")
    opcao = input("Digite sua opção: ")
    
    if opcao == "":
        executar_arquivo_usb(porta_usb)
    elif opcao == "1":
        porta_usb = escolher_porta_usb()
        executar_arquivo_usb(porta_usb)
    elif opcao == "0":
        break  # Finalizar a execução
    else:
        print("Opção inválida. Execução finalizada.")
        break
