#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <string.h>

#define KEY 1234

struct Nachricht // wird über Queue geschickt
{
    long mtype; // 1 Anfrage vom Client, 2 Antwort vom Server
    char text[200];
};

struct ListEintrag
{
    long mtype;
    char text[450];
};

#define MAX_MESSAGES 100

struct Message // Listeneintrag im Speicher des Servers
{
    char request[200];
    char response[200];
};

struct Message messages[MAX_MESSAGES];
int message_count = 0;

int main(void)
{
    // Queue erstellen
    int msgid = msgget(KEY, IPC_CREAT | 0666);

    if (msgid == -1)
    {
        perror("msgget");
        return 1;
    }

    printf("Server gestartet, Queue-ID: %d\n", msgid);

    while (1)
    {
        struct Nachricht anfrage;

        // Warten auf Nachricht vom Client (mtype=1)
        if (msgrcv(msgid, &anfrage, sizeof(anfrage.text), 1, 0) == -1)
        {
            perror("msgrcv");
            continue;
        }

        printf("Empfangen: %s\n", anfrage.text);

        char command[20];
        int b, c;
        struct Nachricht antwort;
        antwort.mtype = 2; // Antworten haben mtype=2

        if (sscanf(anfrage.text, "%19s", command) == 1)
        {
            if (strcmp(command, "ADD") == 0)
            {
                if (sscanf(anfrage.text, "%*s %d %d", &b, &c) == 2)
                {
                    int extra;
                    char meldung[60] = "";
                    if (sscanf(anfrage.text, "%*s %*d %*d %d", &extra) == 1)
                    {
                        snprintf(meldung, sizeof(meldung),
                                 "ADD: Nur die ersten zwei Argumente werden ausgewertet.\n");
                    }

                
                    snprintf(antwort.text, sizeof(antwort.text),
                             "%sRESULT %d", meldung, b + c);

            

                    snprintf(messages[message_count].request,
                             sizeof(messages[message_count].request),
                             "%s", anfrage.text);

                    snprintf(messages[message_count].response,
                             sizeof(messages[message_count].response),
                             "%s", antwort.text);

                    message_count++;
                }
                else
                {
                    snprintf(antwort.text, sizeof(antwort.text),
                             "Ungültige Eingabe für ADD");

                    snprintf(messages[message_count].request,
                             sizeof(messages[message_count].request),
                             "%s", anfrage.text);

                    snprintf(messages[message_count].response,
                             sizeof(messages[message_count].response),
                             "%s", antwort.text);

                    message_count++;
                }
            }
            else if (strcmp(command, "PRIME") == 0)
            {
                int number;
                char extra[100];
                char meldung[50] = "";

                if (sscanf(anfrage.text, "%*s %d %99s", &number, extra) == 2)
                {
                    snprintf(meldung, sizeof(meldung),
                             "PRIME: Nur das erste Argument wird ausgewertet.\n");
                    snprintf(messages[message_count].request,
                             sizeof(messages[message_count].request),
                             "%s", anfrage.text);

                    snprintf(messages[message_count].response,
                             sizeof(messages[message_count].response),
                             "%s", antwort.text);

                    message_count++;
                }

                if (sscanf(anfrage.text, "%*s %d", &number) == 1)
                {
                    int is_prime = 1;

                    if (number < 2)
                    {
                        is_prime = 0;
                    }
                    else
                    {
                        for (int i = 2; i * i <= number; i++)
                        {
                            if (number % i == 0)
                            {
                                is_prime = 0;
                                break;
                            }
                        }
                    }

                    if (is_prime)
                    {
                        snprintf(antwort.text, sizeof(antwort.text),
                                 "%sPRIME YES", meldung);
                        snprintf(messages[message_count].request,
                                 sizeof(messages[message_count].request),
                                 "%s", anfrage.text);

                        snprintf(messages[message_count].response,
                                 sizeof(messages[message_count].response),
                                 "%s", antwort.text);

                        message_count++;
                    }
                    else
                    {
                        snprintf(antwort.text, sizeof(antwort.text),
                                 "%sPRIME NO", meldung);
                        snprintf(messages[message_count].request,
                                 sizeof(messages[message_count].request),
                                 "%s", anfrage.text);

                        snprintf(messages[message_count].response,
                                 sizeof(messages[message_count].response),
                                 "%s", antwort.text);

                        message_count++;
                    }
                }
                else
                {
                    snprintf(antwort.text, sizeof(antwort.text),
                             "PRIME benoetigt eine Zahl als Argument");
                    snprintf(messages[message_count].request,
                             sizeof(messages[message_count].request),
                             "%s", anfrage.text);

                    snprintf(messages[message_count].response,
                             sizeof(messages[message_count].response),
                             "%s", antwort.text);

                    message_count++;
                }
            }

            else if (strcmp(command, "LIST") == 0)
            {
                // Liste zeilenweise als einzelne Nachrichten senden
                for (int i = 0; i < message_count; i++)
                {
                    struct ListEintrag eintrag;
                    eintrag.mtype = 2;
                    snprintf(eintrag.text, sizeof(eintrag.text),
                             "ENTRY %d: %s | %s",
                             i,
                             messages[i].request,
                             messages[i].response);
                    msgsnd(msgid, &eintrag, sizeof(eintrag.text), 0);
                }

                struct ListEintrag ende;
                ende.mtype = 2;
                snprintf(ende.text, sizeof(ende.text), "LIST_END");
                msgsnd(msgid, &ende, sizeof(ende.text), 0);
                continue;
            }

            else if (strcmp(command, "CLEAR") == 0)
            {
                message_count = 0;
                snprintf(antwort.text, sizeof(antwort.text), "OK");
            }
            else
            {
                snprintf(antwort.text, sizeof(antwort.text),
                         "Unbekannter Befehl");
            }
        }

        msgsnd(msgid, &antwort, sizeof(antwort.text), 0);
    }

    return 0;
}
