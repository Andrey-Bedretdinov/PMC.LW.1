#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL

// Определение выводов для сдвигового регистра 74HC595
#define SH_CP_PIN PB0  // Shift register clock (SH_CP)
#define ST_CP_PIN PB2  // Storage register clock (ST_CP)
#define DS_PIN PB1     // Data pin (DS)

// Определение выводов для управления MR и OE
#define MR_PIN PB3     // Подключено к MR
#define OE_PIN PB4     // Подключено к OE

// Определение выводов для управления разрядами индикатора (цифры 1, 2, 3, 4)
#define DIGIT_1 PC5    // Подключено к правому разряду 1 (единицы)
#define DIGIT_2 PC4    // Подключено к правому разряду 2 (десятки)
#define DIGIT_3 PC3    // Подключено к правому разряду 3 (сотни)
#define DIGIT_4 PC2    // Подключено к правому разряду 4 (тысячи)

// Функция для отправки байта в сдвиговый регистр 74HC595
void shiftOut(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        // Передача данных на DS (Q7)
        if (data & (1 << (7 - i))) {
            PORTB |= (1 << DS_PIN);
        } else {
            PORTB &= ~(1 << DS_PIN);
        }
        // Подача импульса на SH_CP (сдвиг)
        PORTB |= (1 << SH_CP_PIN);
        _delay_us(10);
        PORTB &= ~(1 << SH_CP_PIN);
    }
    // Подача импульса на ST_CP (запись данных на выход)
    PORTB |= (1 << ST_CP_PIN);
    _delay_us(10);
    PORTB &= ~(1 << ST_CP_PIN);
}

// Инициализация MR и OE
void init_shift_register_control() {
    // Настроить PB4 и PB5 как выходы для управления MR и OE
    DDRB |= (1 << MR_PIN) | (1 << OE_PIN);

    // Установить MR в логический 1 (сброс отключен)
    PORTB |= (1 << MR_PIN);

    // Установить OE в логический 0 (выходы включены)
    PORTB &= ~(1 << OE_PIN);
}

// Инициализация управления разрядами индикатора
void init_display_control() {
    // Настроить PC2, PC3, PC4, PC5 как выходы для управления разрядами
    DDRC |= (1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4);
}

// Функция для отображения одного разряда (цифры) на индикаторе
void select_digit(uint8_t digit) {
    // Сначала отключаем все разряды
    PORTC &= ~((1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4));

    // Включаем только один выбранный разряд
    switch (digit) {
        case 1:
            PORTC |= (1 << DIGIT_1);  // Единицы
            break;
        case 2:
            PORTC |= (1 << DIGIT_2);  // Десятки
            break;
        case 3:
            PORTC |= (1 << DIGIT_3);  // Сотни
            break;
        case 4:
            PORTC |= (1 << DIGIT_4);  // Тысячи
            break;
        default:
            break;
    }
}

// Функция для отображения значений на индикаторе
void display_value(uint16_t value) {
    // Массив с отображениями цифр на анодный семисегментный индикатор
    uint8_t digits[10] = {
        0b11000000,  // 0: A, b, c, d, e, f
        0b11111001,  // 1: b, c
        0b10100100,  // 2: A, b, d, e, g
        0b10110000,  // 3: A, b, c, d, g
        0b10011001,  // 4: b, c, f, g
        0b10010010,  // 5: A, c, d, f, g
        0b10000010,  // 6: A, c, d, e, f, g
        0b11111000,  // 7: A, b, c
        0b10000000,  // 8: A, b, c, d, e, f, g
        0b10010000   // 9: A, b, c, d, f, g
    };

    // Отображение цифр поочередно на 4 разрядах (от единиц к тысячам)
    select_digit(1);  // Выбираем разряд 1 (единицы)
    shiftOut(digits[value % 10]);  // Единицы
    _delay_ms(5);  // Небольшая задержка для отображения

    select_digit(2);  // Выбираем разряд 2 (десятки)
    shiftOut(digits[(value / 10) % 10]);  // Десятки
    _delay_ms(5);  // Небольшая задержка

    select_digit(3);  // Выбираем разряд 3 (сотни)
    shiftOut(digits[(value / 100) % 10]);  // Сотни
    _delay_ms(5);  // Небольшая задержка

    select_digit(4);  // Выбираем разряд 4 (тысячи)
    shiftOut(digits[(value / 1000) % 10]);  // Тысячи
    _delay_ms(5);  // Небольшая задержка
}

int main(void) {
    // Инициализация пинов для работы со сдвиговым регистром
    DDRB |= (1 << SH_CP_PIN) | (1 << ST_CP_PIN) | (1 << DS_PIN);

    // Инициализация MR и OE
    init_shift_register_control();

    // Инициализация управления разрядами
    init_display_control();

    uint16_t number = 1;  // Начальное значение для вывода

    // Основной цикл
    while (number <= 1000) {
        display_value(number);  // Вывод текущего числа на индикатор
        number++;               // Увеличиваем значение
    }

    while (1) {
        // Бесконечный цикл после завершения вывода чисел
    }
}
