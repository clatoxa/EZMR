#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#define MAX_MESSAGES 100
#define MAX_MESSAGE_LENGTH 200

struct Message
{
    char request[100];
    char response[100];
};

struct Message messages[MAX_MESSAGES];
int message_count = 0;
int peer_socket = -1;

void *accept_connections(void *arg)
{
    int server = *(int *)arg;

    while (1)
    {
        int peer = accept(server, NULL, NULL);

        if (peer == -1)
        {
            perror("accept");
            continue;
        }

        printf("\nPeer verbunden\n");

        while (1)
        {

            char buffer[100];

            int n = recv(peer, buffer, sizeof(buffer) - 1, 0);

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

                        send(peer, antwort, strlen(antwort), 0);
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

                        send(peer, antwort, strlen(antwort), 0);
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

                        send(peer, antwort, strlen(antwort), 0);
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

                        send(peer, antwort, strlen(antwort), 0);
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

                        send(peer, antwort, strlen(antwort), 0);
                    }

                    // Ende der Liste kennzeichnen
                    char ende[] = "LIST_END\n";
                    send(peer, ende, strlen(ende), 0);
                }

                else if (strcmp(command, "CLEAR") == 0)
                {
                    message_count = 0;

                    char antwort[] = "OK";

                    send(peer, antwort, strlen(antwort), 0);
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

                    send(peer, antwort, strlen(antwort), 0);
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

                send(peer, antwort, strlen(antwort), 0);
            }
        }
        close(peer);
    }

    return NULL;
}

int connect_to_peer(int port)
{
    int peer = socket(AF_INET, SOCK_STREAM, 0);

    if (peer == -1)
    {
        perror("socket");
        return -1;
    }

    struct sockaddr_in address = {0};

    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(peer,
                (struct sockaddr *)&address,
                sizeof(address)) == -1)
    {
        perror("connect");
        close(peer);
        return -1;
    }

    printf("Mit Peer auf Port %d verbunden\n", port);

    return peer;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Verwendung: %s <port>\n", argv[0]);
        return 1;
    }

    int port = atoi(argv[1]);

    /*
     * Socket zum Empfangen
     */
    int server = socket(AF_INET, SOCK_STREAM, 0);

    if (server == -1)
    {
        perror("socket");
        return 1;
    }

    struct sockaddr_in address = {0};

    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(server,
             (struct sockaddr *)&address,
             sizeof(address)) == -1)
    {
        perror("bind");
        return 1;
    }

    if (listen(server, 5) == -1)
    {
        perror("listen");
        return 1;
    }

    printf("Peer läuft auf Port %d\n", port);

    /*
     * Thread für eingehende Verbindungen
     */
    pthread_t thread;

    if (pthread_create(&thread,
                       NULL,
                       accept_connections,
                       &server) != 0)
    {
        perror("pthread_create");
        return 1;
    }

    /*
     * Hauptthread:
     * Hier können wir später Benutzerkommandos
     * und ausgehende Verbindungen behandeln.
     */

    while (1)
    {
        char input[100];

        printf("> ");
        fflush(stdout);

        if (fgets(input, sizeof(input), stdin) == NULL)
        {
            break;
        }

        input[strcspn(input, "\n")] = '\0';

        if (strlen(input) == 0)
        {
            printf("Bitte ein Kommando eingeben.\n");
            continue;
        }

        if (strcmp(input, "exit") == 0)
        {
            break;
        }

        if (strncmp(input, "CONNECT ", 8) == 0)
        {
            int port;

            if (sscanf(input, "CONNECT %d", &port) == 1)
            {
                peer_socket = connect_to_peer(port);
            }
            else
            {
                printf("Verwendung: CONNECT <port>\n");
            }
        }
        else
        {
            if (peer_socket == -1)
            {
                printf("Noch nicht mit einem Peer verbunden.\n");
                continue;
            }

            // Befehl an den anderen Peer senden
            if (send(peer_socket, input, strlen(input), 0) == -1)
            {
                perror("send");
                continue;
            }

            // Antwort vom anderen Peer empfangen
            char buffer[500];

            int n = recv(peer_socket, buffer, sizeof(buffer) - 1, 0);

            if (n == -1)
            {
                perror("recv");
                continue;
            }

            if (n == 0)
            {
                printf("Peer hat die Verbindung geschlossen.\n");
                close(peer_socket);
                peer_socket = -1;
                continue;
            }
            buffer[n] = '\0';

            printf("Peer: %s\n", buffer);
        }
    }

    close(server);

    return 0;
}
