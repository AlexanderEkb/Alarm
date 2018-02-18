#include "stm32f0xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "protocol.h"
#include "ui.h"
#include "display.h"
#include "buttons.h"
#include "power.h"
#include "radio.h"
#include "dispatcher.h"

#define QUEUE_LENGTH        (16)
#define AUTO_SHUTDOWN_TIME  (10)

#define FLAG_SUBMENU        (0x00000001)
#define FLAG_CHECKBOX       (0x00000002)
#define FLAG_RADIO          (0x00000004)
#define FLAG_NUMBER         (0x00000008)
#define FLAG_NUMBER_EX      (0x40000008)
#define FLAG_TIME           (0x00000010)
#define FLAG_TEMP			(0x00000020)
#define FLAG_ACTION         (0x00000040)
#define FLAG_READONLY       (0x20000000)
#define FLAG_THE_LAST		(0x80000000)

typedef struct {
    UI_Event_t event;
    uint32_t param0;
    uint32_t param1;
} UI_Notification_t;

typedef enum {
    STANDBY = 0,
    PWRON,
    SPLASH,
    ALARM,
    MENU,
} UIState_t;

typedef struct {
    char *name;
    void *data;
    uint32_t flags;
} MenuItem_t;

typedef struct {
    UIState_t state;
    uint32_t counter;
    uint32_t timestamp;

    void *currentMenu;
    uint32_t currentMenuItem;

    TaskHandle_t self;
} UI_Data_t;

static const __ro_placement MenuItem_t Stub[] = {
    {"Назад", (void *)0, FLAG_THE_LAST}
};

static const __ro_placement MenuItem_t AutostartMenu[] = {
    {"По темп-ре", (void *)0, FLAG_CHECKBOX},
    {"По времени", (void *)0, FLAG_CHECKBOX | FLAG_THE_LAST}
};

static const __ro_placement MenuItem_t SettingsMenu[] = {
	{"Время", (void *)&Stub, 0},
    {"Турботаймер", (void *)&Stub, FLAG_NUMBER_EX},
    {"Период А/З", (void *)&Stub, FLAG_NUMBER},
    {"T\xb8А/З", (void *)0, FLAG_TEMP | FLAG_THE_LAST}
};

static const __ro_placement MenuItem_t StatsMenu[] = {
	{"Двери", (void *)&Stub, FLAG_CHECKBOX | FLAG_READONLY},
	{"Зажигание", (void *)&Stub, FLAG_CHECKBOX | FLAG_READONLY},
	{"Парковка", (void *)&Stub, FLAG_CHECKBOX | FLAG_READONLY},
	{"Свечи", (void *)&Stub, FLAG_CHECKBOX | FLAG_READONLY},
	{"220 V", (void *)&Stub, FLAG_CHECKBOX | FLAG_READONLY},
	{"T\xb8двиг.", (void *)&Stub, FLAG_TEMP | FLAG_READONLY},
	{"RPM", (void *)&Stub, FLAG_CHECKBOX | FLAG_READONLY | FLAG_THE_LAST}
};

static const __ro_placement MenuItem_t MainMenu[] = {
    {"Автозапуск", (void *)&AutostartMenu, FLAG_SUBMENU},
    {"Настройки", (void *)&SettingsMenu, FLAG_SUBMENU},
    {"Состояние", (void *)&StatsMenu, FLAG_SUBMENU | FLAG_THE_LAST}
};

static __no_init struct {
	uint32_t ptr;
	struct {
		MenuItem_t *menu;
		uint32_t pos;
	} stack[10];
} MenuStack;

static QueueHandle_t queue;
static const __packed __ro_placement uint8_t numbers[2][80] = {
    {0xf0,0x08,0x04,0x04,0x04,0x08,0xf0,0x00,0x00,0x40,0x20,0x10,0xfc,0x00,0x00,0x00,
    0x30,0x08,0x04,0x04,0x04,0x08,0xf0,0x00,0x10,0x08,0x04,0x04,0x04,0x88,0x70,0x00,
    0x00,0x00,0x80,0x40,0x30,0xfc,0x00,0x00,0xe0,0xdc,0x44,0x44,0x44,0x84,0x04,0x00,
    0xf0,0x08,0x84,0x84,0x84,0x04,0x18,0x00,0x04,0x04,0x04,0x84,0x64,0x14,0x0c,0x00,
    0x70,0x88,0x04,0x04,0x04,0x88,0x70,0x00,0xf0,0x08,0x04,0x04,0x04,0x08,0xf0,0x00},
    {0x1f,0x20,0x40,0x40,0x40,0x20,0x1f,0x00,0x00,0x00,0x00,0x00,0x7f,0x00,0x00,0x00,
    0x60,0x50,0x48,0x44,0x42,0x41,0x40,0x00,0x18,0x20,0x40,0x41,0x41,0x22,0x1c,0x00,
    0x0c,0x0a,0x09,0x08,0x08,0x7f,0x08,0x00,0x19,0x20,0x40,0x40,0x40,0x20,0x1f,0x00,
    0x1f,0x21,0x40,0x40,0x40,0x21,0x1e,0x00,0x00,0x70,0x0e,0x01,0x00,0x00,0x00,0x00,
    0x1c,0x22,0x41,0x41,0x41,0x22,0x1c,0x00,0x30,0x41,0x42,0x42,0x42,0x21,0x1f,0x00}
};
static const __packed __ro_placement uint8_t antenna[2][16] = {
    {0x04,0x08,0x10,0xfe,0x10,0x08,0x04,0x00,0x04,0x08,0x10,0xfe,0x10,0x08,0xc4,0xc0},
    {0x00,0x00,0x00,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0x00,0x00,0x6f,0x6f}
};

static __no_init UI_Data_t UI_Data;

#include "logo.inc"

static void UI_PwrOn();
static void UI_Splash();
static void UI_DrawLogo();
static void UI_Standby(UI_Notification_t *event);
static void UI_Alarm(UI_Notification_t *event);
static void UI_Menu(UI_Notification_t *event);
static void UI_ComposeMenuScreen();
static void UI_ComposeMenuItem(char *buffer, MenuItem_t *menuItem, uint32_t n);
static void UI_UpdateClock();
static void UI_UpdateLinkStatus();
static void UI_UpdateMenu();
static void UI_Submenu();
static void UI_GoUpper();

void UI_Init() {
    queue = xQueueCreate(QUEUE_LENGTH, sizeof(UI_Notification_t));
    Buttons_Init();
    if(Power_IfPOR()) {
        UI_Data.state = PWRON;
        UI_Data.counter = 0;
        UI_Data.timestamp = 0;
        UI_Data.currentMenu = (void *)&MainMenu[0];
        UI_Data.currentMenuItem = 0;
        UI_Data.self = xTaskGetHandle("UI");
    } else {
        UI_Data.counter++;
    }
}

void UI_Task(void const * argument) {
    UI_Notification_t event = {UI_EVENT_NONE, 0, 0};

    if(Power_IfPOR()) {
        Power_Lock();
        TM_SSD1306_Init();
        Power_Unlock();
        UI_Data.currentMenu = (void *)&MainMenu;
        UI_Data.currentMenuItem = 0;
        MenuStack.ptr = 0;
    }
    while(1) {
		switch(UI_Data.state) {
		case STANDBY:
			UI_Standby(&event);
			break;
		case PWRON:
			UI_PwrOn();
			break;
		case SPLASH:
			UI_Splash();
			break;
		case ALARM:
			UI_Alarm(&event);
			break;
		case MENU:
			UI_Menu(&event);
			break;
		}
		xQueueReceive(queue, &(event), portMAX_DELAY);
    }
}

void UI_Notify(UI_Event_t event, uint32_t param0, uint32_t param1) {
    UI_Notification_t newEvent;

    newEvent.event = event;
    newEvent.param0 = param0;
    newEvent.param1 = param1;

    xQueueSend(queue, (void *)&newEvent, portMAX_DELAY);
}

void UI_NotifyFromISR(UI_Event_t event, uint32_t param0, uint32_t param1, portBASE_TYPE *sc) {
    UI_Notification_t newEvent;

    newEvent.event = event;
    newEvent.param0 = param0;
    newEvent.param1 = param1;

    xQueueSendFromISR(queue, (void *)&newEvent, sc);
}

ErrorStatus UI_IsActive() {
    return UI_Data.state == MENU?SUCCESS:ERROR;
}

void UI_Suspend() {
	vTaskSuspend(UI_Data.self);
}

void UI_Resume() {
	vTaskResume(UI_Data.self);
}

static void UI_DrawLogo() {
	uint32_t i;
	for(i=0;i<8;i++) {
		TM_SSD1306_UpdateScreen(i, 0, SSD1306_WIDTH, (const char *)&startupLogo[i * SSD1306_WIDTH]);
	}
}

static void UI_Standby(UI_Notification_t *event) {
	Action_t Action = ACTION_NOTHING;
    if(event->event == UI_EVENT_BUTTON) {
        //Sound_Beep();
        switch(event->param1) {
        case BUTTON_LEFT:
            Action = ACTION_DISARM;
            break;
        case BUTTON_UP:
            Action = ACTION_START;
            break;
        case BUTTON_RIGHT:
            Action = ACTION_ARM;;
            break;
        case BUTTON_DOWN:
            Action = ACTION_STOP;
            break;
        case BUTTON_PUSH:
            UI_Data.timestamp = UI_Data.counter;
            UI_Data.state = MENU;
            TM_SSD1306_ClearScreen(0, 7);
            UI_ComposeMenuScreen();
            SSD1306_On();
            break;
        }
        Radio_Notify(Action, 0, 0);
        // Do pause
        //vTaskDelay(100);
        Buttons_Activate();
    } else if(event->event == UI_EVENT_RADIO) {
    	//TODO: Define some reaction to radio events.
    }
}

static void UI_PwrOn() {
    Power_Lock();
    UI_DrawLogo();
    SSD1306_On();
    UI_Data.state = SPLASH;
    Power_Unlock();
}

static void UI_Splash() {
    if(UI_Data.counter >= 2) {
        SSD1306_Off();
        UI_Data.state = STANDBY;
    }
}

static void UI_Alarm(UI_Notification_t *event) {
}

static void UI_Menu(UI_Notification_t *event) {
	uint32_t flags;
	if(event->event == UI_EVENT_BUTTON) {
		UI_Data.timestamp = UI_Data.counter;
		//Sound_Beep();
		switch(event->param1) {
		case BUTTON_LEFT:
			if(UI_Data.currentMenuItem) UI_Data.currentMenuItem--;
			break;
		case BUTTON_UP:
			UI_Submenu();
			break;
		case BUTTON_RIGHT:
			flags = ((MenuItem_t *)UI_Data.currentMenu + UI_Data.currentMenuItem)->flags;
			if(!(flags & FLAG_THE_LAST))
				UI_Data.currentMenuItem++;
			break;
		case BUTTON_DOWN:
			UI_GoUpper();
			break;
		case BUTTON_PUSH:
			break;
		}
		Buttons_Activate();
	} else if(event->event == UI_EVENT_RADIO) {
		//TODO: Define some reaction to radio events.
	}
    UI_ComposeMenuScreen();
    if(UI_Data.timestamp + AUTO_SHUTDOWN_TIME == UI_Data.counter) {
        UI_Data.state = STANDBY;
        SSD1306_Off();
    }
}

static void UI_ComposeMenuScreen() {
    // TODO: TM_SSD1306_Fill(SSD1306_COLOR_BLACK);
    UI_UpdateClock();
    UI_UpdateLinkStatus();
    // Update bat status
    // Update alarm status
    // Update menu
    UI_UpdateMenu();
    //TODO: Draw!
}

static void UI_UpdateClock() {
    uint32_t hour, min, time, digit;
    uint32_t i, j;
    char upper[32];
    char lower[32];

    Power_Lock();
    hour = LL_RTC_TIME_GetHour(RTC);
    min = LL_RTC_TIME_GetMinute(RTC);

    time = (hour << 8) | min;
//    x = 128 - 4 * 8;

    for(i=0; i<4; i++) {
        digit = (time & 0x0000f000) >> 12;
        time <<= 4;
        for(j=0; j<8; j++) {
        	upper[i*8+j] = numbers[0][digit*8 + j];
        	lower[i*8+j] = numbers[1][digit*8 + j];
        }
    }
    TM_SSD1306_UpdateScreen(0, 96, 32, &upper[0]);
    TM_SSD1306_UpdateScreen(1, 96, 32, &lower[0]);
    Power_Unlock();
}

static void UI_UpdateLinkStatus() {
    uint32_t shift;

    Power_Lock();
    shift = Radio_GetLinkStatus() == SUCCESS?0:8;

    TM_SSD1306_UpdateScreen(0, 0, 8, (const char *)&antenna[0][shift]);
    TM_SSD1306_UpdateScreen(1, 0, 8, (const char *)&antenna[1][shift]);

    Power_Unlock();
}

static void UI_UpdateMenu() {
	char line[16];
	uint32_t lineCnt = 0;
	MenuItem_t *menuItem = UI_Data.currentMenu;

	while(1) {
		UI_ComposeMenuItem(line, menuItem, lineCnt);
		SSD1306_DrawLine(line, lineCnt + 2, UI_Data.currentMenuItem == lineCnt);
		lineCnt++;
		if((lineCnt >= 8) || (menuItem->flags & FLAG_THE_LAST))
			break;
		menuItem++;
	}
}

static void UI_ComposeMenuItem(char *buffer, MenuItem_t *menuItem, uint32_t n) {
	char symbol;

	memset(buffer, ' ', SSD1306_CHR_WIDTH);
	strncpy(((void *)&buffer[0]), menuItem->name, 11);
	if(menuItem->flags & FLAG_SUBMENU) {
		symbol = 0x1a;
	} else if(menuItem->flags & FLAG_CHECKBOX) {
		symbol = 0x07;
	} else {
		symbol = ' ';
	}
	buffer[12] = symbol;
}

static void UI_Submenu() {
	MenuItem_t *item = ((MenuItem_t *)UI_Data.currentMenu) + UI_Data.currentMenuItem;

	if(item->flags & FLAG_SUBMENU) {
        TM_SSD1306_ClearScreen(1, 7);
		MenuStack.stack[MenuStack.ptr].menu = UI_Data.currentMenu;
		MenuStack.stack[MenuStack.ptr].pos = UI_Data.currentMenuItem;
		MenuStack.ptr++;

		UI_Data.currentMenu = item->data;
		UI_Data.currentMenuItem = 0;
	}
}

static void UI_GoUpper() {
	if(MenuStack.ptr) {
		MenuStack.ptr--;

        TM_SSD1306_ClearScreen(1, 7);
		UI_Data.currentMenu = MenuStack.stack[MenuStack.ptr].menu;
		UI_Data.currentMenuItem = MenuStack.stack[MenuStack.ptr].pos;
	}
}
