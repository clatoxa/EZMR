#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>

#define MAX_MESSAGES 100
#define MAX_MESSAGE_LENGTH 200

struct Message
{
    char request[100];
    char response[100];
};

struct Message messages[MAX_MESSAGES];
int message_count = 0;

int main(void)
{
    int a = socket(AF_INET, SOCK_STREAM, 0);

    if (a == -1)
    {
        perror("Fehler");
    }
    else
    {
        printf("sucess\n");
    }

    struct sockaddr_in address = {0};

    address.sin_family = AF_INET;
    address.sin_port = htons(5071);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(a, (struct sockaddr *)&address, sizeof(address)) == -1)
    {
        perror("bind");
    }
    else
    {
        printf("bind erfolgreich\n");
    }

    if (listen(a, 5) == -1)
    {
        perror("listen");
    }
    else
    {
        printf("Server wartet auf Verbindungen\n");
    }

    while (1)
    {
        int client = accept(a, NULL, NULL);

        if (client == -1)
        {
            perror("accept");
            continue;
        }

        printf("Client verbunden\n");

        while (1)
        {

            char buffer[100];

            int n = recv(client, buffer, sizeof(buffer) - 1, 0);

            if (n == -1)
            {
                perror("recv");
                break;
            }
            else if (n == 0)
            {
                printf("Server hat die Verbindung geschlossen.\n");
                break;
            }

            buffer[n] = '\0';

            printf("Empfangen: %s\n", buffer);

            char command[20];
            int b, c;

            if (sscanf(buffer, "%19s", command) == 1)
            {

                if (strcmp(command, "ADD") == 0)
                {
                    if (sscanf(buffer, "%*s %d %d", &b, &c) == 2)
                    {
                        int result = b + c;

                        char antwort[100];
                        snprintf(antwort, sizeof(antwort),
                                 "RESULT %d", result);

                        snprintf(messages[message_count].request,
                                 sizeof(messages[message_count].request),
                                 "%s",
                                 buffer);

                        snprintf(messages[message_count].response,
                                 sizeof(messages[message_count].response),
                                 "%s",
                                 antwort);

                        message_count++;

                        send(client, antwort, strlen(antwort), 0);
                    }
                    else
                    {
                        char antwort[] = "Ungültige Eingabe für ADD";

                        snprintf(messages[message_count].request,
                                 sizeof(messages[message_count].request),
                                 "%s",
                                 buffer);

                        snprintf(messages[message_count].response,
                                 sizeof(messages[message_count].response),
                                 "%s",
                                 antwort);

                        message_count++;

                        send(client, antwort, strlen(antwort), 0);
                    }
                }

                else if (strcmp(command, "PRIME") == 0)
                {
                    int number;
                    char extra[100];
                    char meldung[50];
                    if (sscanf(buffer, "%*s %d %99s", &number, extra) == 2)
                    {

                        snprintf(meldung, sizeof(meldung), "PRIME: Nur das erste Argument wird ausgewertet.\n");
                    }

                    if (sscanf(buffer, "%*s %d", &number) == 1)
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

                        char antwort[100];

                        if (is_prime)
                        {
                            snprintf(antwort, sizeof(antwort),
                                     "%sPRIME YES", meldung);
                        }
                        else
                        {
                            snprintf(antwort, sizeof(antwort),
                                     "%sPRIME NO", meldung);
                        }

                        /*
                         * Request und Response speichern
                         */
                        snprintf(messages[message_count].request,
                                 sizeof(messages[message_count].request),
                                 "%s",
                                 buffer);

                        snprintf(messages[message_count].response,
                                 sizeof(messages[message_count].response),
                                 "%s",
                                 antwort);

                        message_count++;

                        send(client, antwort, strlen(antwort), 0);
                    }
                    else
                    {
                        char antwort[] = "PRIME benätigt eine Zahl als Argument";

                        snprintf(messages[message_count].request,
                                 sizeof(messages[message_count].request),
                                 "%s",
                                 buffer);

                        snprintf(messages[message_count].response,
                                 sizeof(messages[message_count].response),
                                 "%s",
                                 antwort);

                        message_count++;

                        send(client, antwort, strlen(antwort), 0);
                    }
                }

                else if (strcmp(command, "LIST") == 0)
                {
                    printf("LIST wurde angefordert\n");

                    for (int i = 0; i < message_count; i++)
                    {
                        char antwort[250];

                        snprintf(antwort, sizeof(antwort),
                                 "ENTRY REQUEST: %s | RESPONSE: %s\n",
                                 messages[i].request,
                                 messages[i].response);

                        send(client, antwort, strlen(antwort), 0);
                    }

                    // Ende der Liste kennzeichnen
                    char ende[] = "LIST_END\n";
                    send(client, ende, strlen(ende), 0);
                }

                else if (strcmp(command, "CLEAR") == 0)
                {
                    message_count = 0;

                    char antwort[] = "OK";

                    send(client, antwort, strlen(antwort), 0);
                }

                else
                {
                    char antwort[] = "Unbekannter Befehl";

                    snprintf(messages[message_count].request,
                             sizeof(messages[message_count].request),
                             "%s",
                             buffer);

                    snprintf(messages[message_count].response,
                             sizeof(messages[message_count].response),
                             "%s",
                             antwort);

                    message_count++;

                    send(client, antwort, strlen(antwort), 0);
                }
            }

            else
            {
                char antwort[] = "Ungültige Eingabe";

                snprintf(messages[message_count].request,
                         sizeof(messages[message_count].request),
                         "%s",
                         buffer);

                snprintf(messages[message_count].response,
                         sizeof(messages[message_count].response),
                         "%s",
                         antwort);

                message_count++;

                send(client, antwort, strlen(antwort), 0);
            }
        }

        close(client);
    }

    return 0;
}
