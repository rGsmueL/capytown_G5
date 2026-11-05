
#!/usr/bin/env python3

import argparse


def main():
    parser = argparse.ArgumentParser(description='Comparar distancia real y odometría para calibrar el radio de rueda.')

    parser.add_argument('d_real', type=float, help='Distancia física medida con cinta (m)')
    parser.add_argument('d_odom', type=float, help='Distancia registrada por /odom (m)')
    parser.add_argument( '--radio-actual-mm', type=float, default=None, help='Radio configurado actualmente (mm)')

    args = parser.parse_args()

    d_real = args.d_real
    d_odom = args.d_odom

    if d_real <= 0 or d_odom <= 0:
        parser.error('Las dos distancias deben ser mayores que cero.')

    # Error relativo respecto a la distancia física.
    error = abs(d_real - d_odom) / d_real * 100

    # Factor de corrección del radio.
    factor = d_real / d_odom

    print('\n--- RESULTADOS DE CALIBRACIÓN ---')
    print(f'Distancia física:    {d_real:.4f} m')
    print(f'Distancia odometría: {d_odom:.4f} m')
    print(f'Error absoluto:      {abs(d_real-d_odom):.4f} m')
    print(f'Error porcentual:    {error:.2f} %')
    print(f'Factor de corrección: {factor:.6f}')

    if error <= 2.0:
        print('Resultado: CUMPLE el límite del 2 %.')
    else:
        print('Resultado: NO CUMPLE el límite del 2 %.')

    if d_odom < d_real:
        print('La odometría registra menos distancia que la real.')
        print('El radio configurado probablemente es demasiado pequeño.')
    elif d_odom > d_real:
        print('La odometría registra más distancia que la real.')
        print('El radio configurado probablemente es demasiado grande.')
    else:
        print('Las distancias coinciden.')

    if args.radio_actual_mm is not None:
        r_actual = args.radio_actual_mm

        if r_actual <= 0:
            parser.error('El radio actual debe ser mayor que cero.')

        r_nuevo = r_actual * factor
        diametro_nuevo = 2 * r_nuevo

        print('\n--- RADIO ESTIMADO CORREGIDO ---')
        print(f'Radio actual:        {r_actual} mm')
        print(f'Radio recomendado:   {r_nuevo} mm')
        print(f'Diámetro estimado:   {diametro_nuevo} mm')
        print('Actualiza el parámetro y vuelve a medir.')
        print('No cambies b_eff durante esta calibración.')

    print('\nNota: valida el resultado con nuevas corridas físicas.')


if __name__ == '__main__':
    main()
