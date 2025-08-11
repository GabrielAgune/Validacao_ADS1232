#include "cli_driver.h"
#include "dwin_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// Definições
#define CLI_BUFFER_SIZE 128

// Protótipos de Funções de Comando
static void Process_Command(void);
static void Cmd_Help(char* args);
static void Cmd_Dwin(char* args);
static void Handle_Dwin_PIC(char* sub_args);
static void Handle_Dwin_INT(char* sub_args);
static void Handle_Dwin_INT32(char* sub_args);
static void Handle_Dwin_RAW(char* sub_args);
static uint8_t hex_char_to_value(char c);

// Variáveis Estáticas
static UART_HandleTypeDef* s_huart_debug = NULL;
static char s_cli_buffer[CLI_BUFFER_SIZE];
static uint16_t s_cli_buffer_index = 0;
static bool s_command_ready = false;

// Tabela de Comandos Principal
typedef struct { const char* name; void (*handler)(char* args); } cli_command_t;
static const cli_command_t s_command_table[] = {
    { "HELP", Cmd_Help }, { "?", Cmd_Help }, { "DWIN", Cmd_Dwin },
};
static const size_t NUM_COMMANDS = sizeof(s_command_table) / sizeof(s_command_table[0]);

// Tabela de Subcomandos DWIN
typedef struct { const char* name; void (*handler)(char* args); } dwin_subcommand_t;
static const dwin_subcommand_t s_dwin_table[] = {
    { "PIC", Handle_Dwin_PIC }, { "INT", Handle_Dwin_INT },
    { "INT32", Handle_Dwin_INT32 }, { "RAW", Handle_Dwin_RAW }
};
static const size_t NUM_DWIN_SUBCOMMANDS = sizeof(s_dwin_table) / sizeof(s_dwin_table[0]);

static const char HELP_TEXT[] =
    "\r\n====================== CLI de Teste DWIN ======================\r\n"
    "| HELP ou ?                | Mostra esta ajuda.                        |\r\n"
    "| DWIN PIC <id>            | Muda a tela (ex: DWIN PIC 1).           |\r\n"
    "| DWIN INT <addr> <val>    | Escreve int16 (ex: DWIN INT 1500 -10).  |\r\n"
    "| DWIN INT32 <addr> <val>  | Escreve int32 (ex: DWIN INT32 1500 40500)|\r\n"
    "| DWIN RAW <hex...>        | Envia bytes hex (ex: DWIN RAW 5A A5...).|\r\n"
    "=================================================================\r\n";

void CLI_Init(UART_HandleTypeDef* debug_huart) {
    s_huart_debug = debug_huart;
    printf("\r\nCLI Pronta. Digite 'HELP' para comandos.\r\n> ");
}

void CLI_Process(void) {
    if (s_command_ready) {
        printf("\r\n");
        Process_Command();
        memset(s_cli_buffer, 0, CLI_BUFFER_SIZE);
        s_cli_buffer_index = 0;
        s_command_ready = false;
        printf("\r\n> ");
    }
}

void CLI_Receive_Char(uint8_t received_char) {
    if (s_command_ready) return;
    if (received_char == '\r' || received_char == '\n') {
        if (s_cli_buffer_index > 0) {
            s_cli_buffer[s_cli_buffer_index] = '\0';
            s_command_ready = true;
        } else {
            printf("\r\n> ");
        }
    } else if (received_char == '\b' || received_char == 127) {
        if (s_cli_buffer_index > 0) {
            s_cli_buffer_index--;
            HAL_UART_Transmit(s_huart_debug, (uint8_t*)"\b \b", 3, 100);
        }
    } else if (s_cli_buffer_index < (CLI_BUFFER_SIZE - 1) && isprint(received_char)) {
        s_cli_buffer[s_cli_buffer_index++] = received_char;
        HAL_UART_Transmit(s_huart_debug, &received_char, 1, 100);
    }
}

// --- FUNÇÃO DE PROCESSAMENTO CORRIGIDA ---
static void Process_Command(void) {
    char* command_str = s_cli_buffer;
    char* args = NULL;

    // Remove espaços em branco no início
    while (isspace((unsigned char)*command_str)) command_str++;

    // Encontra o primeiro espaço para separar o comando dos argumentos
    args = strchr(command_str, ' ');
    if (args != NULL) {
        *args = '\0'; // Termina a string do comando
        args++;       // Move o ponteiro para o início dos argumentos
        while (isspace((unsigned char)*args)) args++; // Remove espaços dos argumentos
        if (*args == '\0') args = NULL; // Se só havia espaços, não há argumentos
    }

    if (*command_str == '\0') return; // Linha vazia

    for (size_t i = 0; i < NUM_COMMANDS; i++) {
        if (strcasecmp(command_str, s_command_table[i].name) == 0) {
            s_command_table[i].handler(args);
            return;
        }
    }
    printf("Comando desconhecido: \"%s\".", command_str);
}

static void Cmd_Help(char* args) { printf("%s", HELP_TEXT); }

static void Cmd_Dwin(char* args) {
    if (args == NULL) { printf("Subcomando DWIN faltando. Use 'HELP'."); return; }
    
    char* sub_cmd = args;
    char* sub_args = NULL;
    
    sub_args = strchr(sub_cmd, ' ');
    if (sub_args != NULL) {
        *sub_args = '\0';
        sub_args++;
        while (isspace((unsigned char)*sub_args)) sub_args++;
        if (*sub_args == '\0') sub_args = NULL;
    }

    for (size_t i = 0; i < NUM_DWIN_SUBCOMMANDS; i++) {
        if (strcasecmp(sub_cmd, s_dwin_table[i].name) == 0) {
            s_dwin_table[i].handler(sub_args);
            return;
        }
    }
    printf("Subcomando DWIN desconhecido: \"%s\"", sub_cmd);
}

static void Handle_Dwin_PIC(char* sub_args) {
    if (sub_args == NULL) { printf("Uso: DWIN PIC <id>"); return; }
    DWIN_Driver_SetScreen(atoi(sub_args));
    printf("Comando enviado: Mudar para tela ID %s", sub_args);
}

static void Handle_Dwin_INT(char* sub_args) {
    if (sub_args == NULL) { printf("Uso: DWIN INT <addr_hex> <valor>"); return; }
    char* val_str = NULL;
    char* addr_str = sub_args;
    val_str = strchr(addr_str, ' ');
    if (val_str == NULL) { printf("Valor faltando."); return; }
    *val_str = '\0'; val_str++;

    uint16_t vp = strtol(addr_str, NULL, 16);
    int16_t val = atoi(val_str);
    DWIN_Driver_WriteInt(vp, val);
    printf("Escrevendo (int16) %d em 0x%04X", val, vp);
}

static void Handle_Dwin_INT32(char* sub_args) {
    if (sub_args == NULL) { printf("Uso: DWIN INT32 <addr_hex> <valor>"); return; }
    char* val_str = NULL;
    char* addr_str = sub_args;
    val_str = strchr(addr_str, ' ');
    if (val_str == NULL) { printf("Valor faltando."); return; }
    *val_str = '\0'; val_str++;

    uint16_t vp = strtol(addr_str, NULL, 16);
    int32_t val = atol(val_str);
    DWIN_Driver_WriteInt32(vp, val);
    printf("Escrevendo (int32) %ld em 0x%04X", (long)val, vp);
}

static uint8_t hex_char_to_value(char c) {
    c = toupper((unsigned char)c);
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return 0xFF; // Erro
}

static void Handle_Dwin_RAW(char* sub_args) {
    if (sub_args == NULL) { printf("Uso: DWIN RAW <byte_hex> ..."); return; }
    uint8_t raw_buffer[CLI_BUFFER_SIZE / 2];
    int byte_count = 0;
    char* ptr = sub_args;
    while (*ptr != '\0' && byte_count < (CLI_BUFFER_SIZE / 2)) {
        while (isspace((unsigned char)*ptr)) ptr++;
        if (*ptr == '\0') break;
        char high_c = *ptr++;
        if (*ptr == '\0' || isspace((unsigned char)*ptr)) { printf("\nErro: Numero impar de caracteres hex."); return; }
        char low_c = *ptr++;
        uint8_t high_v = hex_char_to_value(high_c);
        uint8_t low_v = hex_char_to_value(low_c);
        if (high_v == 0xFF || low_v == 0xFF) { printf("\nErro: Caractere invalido na string hex."); return; }
        raw_buffer[byte_count++] = (high_v << 4) | low_v;
    }
    printf("Enviando %d bytes:", byte_count);
    for(int i = 0; i < byte_count; i++) printf(" %02X", raw_buffer[i]);
    DWIN_Driver_WriteRawBytes(raw_buffer, byte_count);
}