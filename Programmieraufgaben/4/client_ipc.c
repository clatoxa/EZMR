#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <string.h>

#define KEY 1234

struct Nachricht
{
    long mtype;
    char text[200];
};

struct ListEintrag 
{
    long mtype;
    char text[450];
};

int main(void)
{
    // dieselbe Queue öffnen (kein IPC_CREAT – Server muss zuerst starten)
    int msgid = msgget(KEY, 0666);

    if (msgid == -1)
    {
        perror("msgget");
        return 1;
    }

    printf("Verbunden mit Queue-ID: %d\n", msgid);

    char input[200];
    struct Nachricht nachricht;

    while (1)
    {
        printf("> ");
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL)
            break;

        input[strcspn(input, "\n")] = '\0';

        if (strcmp(input, "exit") == 0)
            break;

        // Anfrage senden (mtype=1)
        nachricht.mtype = 1;
        snprintf(nachricht.text, sizeof(nachricht.text), "%s", input);

        if (msgsnd(msgid, &nachricht, sizeof(nachricht.text), 0) == -1)
        {
            perror("msgsnd");
            continue;
        }

        // Antwort empfangen (mtype=2)
        

        if (strcmp(input, "LIST") == 0)
        {
            while (1)
            {   
                struct ListEintrag antwort;
                if (msgrcv(msgid, &antwort, sizeof(antwort.text), 2, 0) == -1)
                {
                    perror("msgrcv");
                    break;
                }

                printf("Server: %s\n", antwort.text);

                if (strcmp(antwort.text, "LIST_END") == 0)
                    break;
            }
        }
        else
        {
            struct Nachricht antwort;
            if (msgrcv(msgid, &antwort, sizeof(antwort.text), 2, 0) == -1)
            {
                perror("msgrcv");
                continue;
            }

            printf("Server: %s\n", antwort.text);
        }

        
    }
return 0;
}