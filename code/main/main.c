
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <util/delay.h>

#define F_CPU 8000000UL  // Частота работы микроконтроллера 8 МГц

// Определение выводов для кнопки и управления режимом
#define BUTTON_PIN PD2  // Кнопка подключена к PD2/INT0
#define FREQ_PIN PB6    // Логическая единица при выводе частоты
#define AMP_PIN PB7     // Логическая единица при выводе амплитуды

// Определение выводов для сдвигового регистра 74HC595
#define SH_CP_PIN PB0  // Shift register clock (SH_CP)
#define ST_CP_PIN PB2  // Storage register clock (ST_CP)
#define DS_PIN PB1     // Data pin (DS)
#define MR_PIN PB3  // Подключено к MR (Master Reset)
#define OE_PIN PB4  // Подключено к OE (Output Enable)


// Определение выводов для управления разрядами индикатора
#define DIGIT_1 PC5  // Подключено к правому разряду 1
#define DIGIT_2 PC4  // Подключено к правому разряду 2
#define DIGIT_3 PC3  // Подключено к правому разряду 3
#define DIGIT_4 PC2  // Подключено к правому разряду 4

// Переменные для частоты и амплитуды
volatile float signal_frequency = 0;
volatile uint16_t signal_amplitude = 0;
volatile uint16_t adc_value = 0;
volatile uint16_t max_value = 0;
volatile uint16_t min_value = 1023;

// Переменная для задания задержки между обновлениями сегментов индикатора
uint8_t segment_delay = 200;  // Задержка в миллисекундах между переключениями разрядов

// Пороговое значение для пересечения "нуля"
#define ZERO_CROSS_THRESHOLD 512  // Эквивалент 2.5В при 10-битном АЦП

// Коэффициент для перевода значений АЦП в вольты
#define ADC_TO_VOLTAGE_COEFF (5.0 / 1023.0)

// Переменные для подсчета частоты
volatile uint16_t zero_cross_count = 0;
volatile uint8_t measurement_done = 0;
volatile uint8_t show_frequency = 1;  // 1 - показывать частоту, 0 - амплитуду

// Массив для отображения цифр на анодном индикаторе
uint8_t digits[10] = {
    0b11000000,  // 0
    0b11111001,  // 1
    0b10100100,  // 2
    0b10110000,  // 3
    0b10011001,  // 4
    0b10010010,  // 5
    0b10000010,  // 6
    0b11111000,  // 7
    0b10000000,  // 8
    0b10010000   // 9
};

// Инициализации UART
void uart_init() {
    uint16_t ubrr_value = 51;  // Для частоты 8 МГц и скорости 9600 бод
    UBRR0H = (ubrr_value >> 8);
    UBRR0L = ubrr_value;
    UCSR0B = (1 << TXEN0);  // Включить передатчик
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 бит данных
}

// Отправка одного байта через UART
void uart_transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// Отправка строки через UART
void uart_print(const char* str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

// Инициализация MR и OE для 74HC595
void init_shift_register_control() {
    DDRB |= (1 << MR_PIN) | (1 << OE_PIN);  // Настроить PB3 и PB4 как выходы

    // Установить MR в логический 1 (нормальная работа, сброс не активен)
    PORTB |= (1 << MR_PIN);

    // Установить OE в логический 0 (включить выходы сдвигового регистра)
    PORTB &= ~(1 << OE_PIN);
}


// Отправка байта в сдвиговый регистр 74HC595
void shiftOut(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & (1 << (7 - i))) {
            PORTB |= (1 << DS_PIN);
        } else {
            PORTB &= ~(1 << DS_PIN);
        }
        // Подача тактового сигнала
        PORTB |= (1 << SH_CP_PIN);
        _delay_us(10);
        PORTB &= ~(1 << SH_CP_PIN);
    }
    // Подача сигнала для обновления выходов регистра
    PORTB |= (1 << ST_CP_PIN);
    _delay_us(10);
    PORTB &= ~(1 << ST_CP_PIN);
}


// Выбор разряда семисегментного индикатора
void select_digit(uint8_t digit) {
    PORTC &= ~((1 << DIGIT_1) | (1 << DIGIT_2) | (1 << DIGIT_3) | (1 << DIGIT_4));
    switch (digit) {
        case 1: PORTC |= (1 << DIGIT_1); break;
        case 2: PORTC |= (1 << DIGIT_2); break;
        case 3: PORTC |= (1 << DIGIT_3); break;
        case 4: PORTC |= (1 << DIGIT_4); break;
        default: break;
    }
}

// Отображение числа на семисегментном индикаторе
void display_value(float value) {
    // Преобразуем значение в строку без округления
    char buffer[8];
    dtostrf(value, 4, 3, buffer);  // Преобразуем значение float в строку формата "XX.XXX"

    uint8_t display_index = 4;  // Индекс для разрядов (начинаем с левого)
    uint8_t digit_index = 0;    // Индекс для символов строки

    // Проходим по каждому символу в строке
    while (buffer[digit_index] != '\0' && display_index > 0) {
        if (buffer[digit_index] >= '0' && buffer[digit_index] <= '9') {
            // Отображаем цифру
            select_digit(display_index);  // Выбираем разряд
            shiftOut(digits[buffer[digit_index] - '0']);  // Отправляем цифру на индикатор
            display_index--;  // Переходим к следующему разряду
        } else if (buffer[digit_index] == '.') {
            // Если встретили точку, добавляем точку к предыдущему разряду
            select_digit(display_index + 1);  // Добавляем точку к предыдущему символу
            shiftOut(digits[buffer[digit_index - 1] - '0'] & ~(1 << 7));  // Включаем точку
        }
        digit_index++;  // Переходим к следующему символу строки
        _delay_ms(segment_delay);  // Задержка для стабильного отображения
    }

    // Если осталось пустое место, выключаем оставшиеся индикаторы
    while (display_index > 0) {
        select_digit(display_index);  // Выбираем разряд
        shiftOut(0xFF);  // Выключаем индикатор (0xFF выключает все сегменты)
        display_index--;
        _delay_ms(segment_delay);
    }
}





// Инициализация АЦП
void adc_init() {
    ADMUX = (1 << REFS0);  // AREF = AVcc, вход ADC0
    ADCSRA = (1 << ADEN)  | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);  // Включить АЦП, прерывания, предделитель 64
}

// Запуск следующего преобразования АЦП
void adc_start_conversion() {
    ADCSRA |= (1 << ADSC);
}

// Обработчик прерывания АЦП
ISR(ADC_vect) {
    // Переменная для хранения предыдущего значения АЦП, используется для отслеживания пересечения через "ноль"
    static uint16_t last_adc_value = 0;

    // Чтение текущего значения с АЦП (в регистре ADC хранится результат последнего преобразования)
    adc_value = ADC;

    // Проверка, является ли текущее значение максимальным за цикл измерений
    if (adc_value > max_value) {
        max_value = adc_value;  // Сохраняем новое максимальное значение
    }

    // Проверка, является ли текущее значение минимальным за цикл измерений
    if (adc_value < min_value) {
        min_value = adc_value;  // Сохраняем новое минимальное значение
    }

    // Если текущее значение АЦП находится выше порога (середина диапазона) и предыдущее значение было ниже,
    // то это означает пересечение через "ноль" (для синусоидального сигнала это ключевая точка для измерения частоты)
    if ((adc_value >= ZERO_CROSS_THRESHOLD) && (last_adc_value < ZERO_CROSS_THRESHOLD)) {
        zero_cross_count++;  // Увеличиваем счетчик пересечений через "ноль"
    }

    // Сохраняем текущее значение как предыдущее для следующего сравнения
    last_adc_value = adc_value;

    // Инициируем следующий цикл преобразования АЦП, чтобы продолжать измерения
    ADCSRA |= (1 << ADSC);  // Устанавливаем бит ADSC (Start Conversion) для запуска нового преобразования
}


// Расчет амплитуды в вольтах
float get_amplitude_in_volts() {
    return (signal_amplitude * ADC_TO_VOLTAGE_COEFF) / 2.0;
}

// Расчет текущего значения в вольтах
float get_current_voltage() {
    return adc_value * ADC_TO_VOLTAGE_COEFF;
}

// Обработчик прерывания таймера 1 (срабатывает раз в 1 секунду)
ISR(TIMER1_COMPA_vect) {
    // Расчет частоты сигнала:
    // Два пересечения через "ноль" означают один полный цикл (синусоида проходит вверх и вниз).
    // Поэтому делим количество пересечений на 2, чтобы получить частоту в герцах.
    signal_frequency = zero_cross_count / 2.0;

    // Расчет амплитуды сигнала:
    // Амплитуда — это разница между максимальным и минимальным значением АЦП, зафиксированным за последний цикл измерений.
    signal_amplitude = max_value - min_value;

    // Сброс счетчика пересечений через "ноль" для нового цикла измерений в следующую секунду.
    zero_cross_count = 0;

    // Сброс максимального и минимального значений для нового цикла измерений.
    max_value = 0;
    min_value = 1023;  // Для 10-битного АЦП максимальное значение — 1023 (это означает "0" для минимума в следующий цикл).

    // Устанавливаем флаг, что измерения завершены и доступны для вывода или дальнейшей обработки.
    measurement_done = 1;
}


// Инициализация таймера 1
void timer1_init() {
    // Настройка таймера 1 на CTC режим с периодом 1 секунда
    TCCR1B |= (1 << WGM12);  // CTC режим
    OCR1A = 15624;  // Для 1 секунды при предделителе 512 (8MHz / 512 / 15625 = 1Hz)
    TCCR1B |= (1 << CS12) | (1 << CS10);  // Предделитель 1024
    TIMSK1 |= (1 << OCIE1A);  // Разрешить прерывание по совпадению
}

// Инициализация кнопки на PD2/INT0
void button_init() {
    DDRD &= ~(1 << BUTTON_PIN);  // PD2 как вход
    PORTD |= (1 << BUTTON_PIN);  // Включить подтягивающий резистор
    EICRA |= (1 << ISC01);  // Прерывание по спаду на INT0
    EIMSK |= (1 << INT0);   // Разрешить прерывание INT0
}

// Обработчик прерывания INT0 (внешнее прерывание от кнопки на PD2)
ISR(INT0_vect) {
    // Переключение режима отображения: 
    // Если сейчас отображается частота, переключаем на амплитуду и наоборот.
    show_frequency = !show_frequency;

    // Если выбрана частота для отображения (show_frequency == 1):
    if (show_frequency) {
        // Включаем вывод на частоту: устанавливаем логическую 1 на выводе FREQ_PIN (PB6).
        PORTB |= (1 << FREQ_PIN);

        // Выключаем вывод амплитуды: сбрасываем логическую 1 на выводе AMP_PIN (PB7).
        PORTB &= ~(1 << AMP_PIN);
    } 
    // Если выбрана амплитуда для отображения (show_frequency == 0):
    else {
        // Включаем вывод на амплитуду: устанавливаем логическую 1 на выводе AMP_PIN (PB7).
        PORTB |= (1 << AMP_PIN);

        // Выключаем вывод частоты: сбрасываем логическую 1 на выводе FREQ_PIN (PB6).
        PORTB &= ~(1 << FREQ_PIN);
    }
}


// Инициализация выводов PB6 и PB7
void init_mode_pins() {
    DDRB |= (1 << FREQ_PIN) | (1 << AMP_PIN);  // Настраиваем PB6 и PB7 как выходы

    // Установить начальное состояние: частота на PB6, амплитуда на PB7
    PORTB |= (1 << FREQ_PIN);  // По умолчанию частота
    PORTB &= ~(1 << AMP_PIN);  // Амплитуда выключена
}

int main(void) {
	
    // Инициализация пинов для работы со сдвиговым регистром и управления MR/OE
    DDRB |= (1 << SH_CP_PIN) | (1 << ST_CP_PIN) | (1 << DS_PIN);
    init_shift_register_control();  // Инициализация MR и OE

    uart_init();          // Инициализация UART
    adc_init();           // Инициализация АЦП
    timer1_init();        // Инициализация таймера 1
    button_init();        // Инициализация кнопки
    init_mode_pins();     // Инициализация выводов PB6 и PB7
    sei();                // Включить глобальные прерывания

    adc_start_conversion();  // Запустить первое преобразование АЦП

    while (1) {
        if (measurement_done) {
            char buffer[32];
            float current_frequency = signal_frequency;
            float current_amplitude = get_amplitude_in_volts();
            float current_voltage = get_current_voltage();

            // Логирование частоты
            uart_print("Frequency: ");
            dtostrf(current_frequency, 6, 2, buffer);
            uart_print(buffer);
            uart_print(" Hz\r\n");

            // Логирование амплитуды
            uart_print("Amplitude: ");
            dtostrf(current_amplitude, 4, 2, buffer);
            uart_print(buffer);
            uart_print(" V\r\n");

            // Логирование текущего напряжения
            uart_print("Current Voltage: ");
            dtostrf(current_voltage, 4, 2, buffer);
            uart_print(buffer);
            uart_print(" V\r\n\r\n");

            measurement_done = 0;  // Сброс флага

            // Определение значения для вывода на семисегментный индикатор
            float value_to_display = show_frequency ? current_frequency : current_amplitude;
            display_value(value_to_display);  // Вывод значения на индикатор
        }
    }
}




