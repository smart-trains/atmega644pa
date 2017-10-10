cmake_minimum_required(VERSION 3.8)
project(atmega644pa)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        ASF/common/boards/user_board/init.c
        ASF/common/boards/user_board/user_board.h
        ASF/common/boards/board.h
        ASF/common/services/clock/mega/sysclk.c
        ASF/common/services/clock/mega/sysclk.h
        ASF/common/services/clock/sysclk.h
        ASF/common/services/delay/mega/cycle_counter.h
        ASF/common/services/delay/delay.h
        ASF/common/services/ioport/mega/ioport.h
        ASF/common/services/ioport/ioport.h
        ASF/common/utils/interrupt/interrupt_avr8.h
        ASF/common/utils/interrupt.h
        ASF/common/utils/parts.h
        ASF/mega/drivers/cpu/mega_reset_cause.h
        ASF/mega/utils/assembler/gas.h
        ASF/mega/utils/preprocessor/mrepeat.h
        ASF/mega/utils/preprocessor/preprocessor.h
        ASF/mega/utils/preprocessor/stringz.h
        ASF/mega/utils/preprocessor/tpaste.h
        ASF/mega/utils/assembler.h
        ASF/mega/utils/compiler.h
        ASF/mega/utils/progmem.h
        ASF/mega/utils/status_codes.h
        config/conf_board.h
        config/conf_clock.h
        asf.h
        base.h
        functions.h
        iom644pa.h
        macros.h
        main.c
        sc18is600.h
        sc18is600_arch.h
        sfr_defs.h)

add_executable(atmega644pa ${SOURCE_FILES})