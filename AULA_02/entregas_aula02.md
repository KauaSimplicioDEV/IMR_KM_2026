Pose 2D é só o "RG" do robô: onde ele tá (x, y) e pra onde ele tá olhando (θ). Todo o resto do código é basicamente ficar atualizando esses 3 números o tempo todo.

Cinemática diferencial é a conta que transforma "quanto cada roda tá girando" em "pra onde o robô vai". Sacada legal: se as duas rodas giram opostas, o robô roda no próprio eixo parado no lugar; se só uma roda gira, ele faz uma curva em torno da roda travada, tipo um pião capenga.

Odometria discreta é o robô tentando adivinhar onde ele tá só de olhar quanto tempo passou (dt) e quão rápido ele mandou as rodas girarem — sem GPS, sem sensor de verdade, só matemática e fé. Por isso no exercício do quadrado ele quase fecha o percurso, mas nunca fecha 100%: cada friozinho de erro no tempo do frame vai se acumulando e no final ele termina um pouquinho torto do ponto de partida.

Go-to-goal é a parte engraçada: o robô olha pro alvo, calcula o ângulo até ele, vê o quanto tá errado (erro_θ) e gira proporcional a esse erro (ω = Kp × erro_θ). Resultado prático: ele primeiro gira feito um bêbado procurando a direção certa, aí sim sai andando na moral até chegar perto o suficiente e parar sozinho.
