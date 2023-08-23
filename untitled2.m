syms c_alpha s_alpha c_beta s_beta c_suita s_suita a d

Tibeta=[1 0 0 a;0 c_alpha -s_alpha 0;0 s_alpha c_alpha 0;0 0 0 1]*[c_beta 0 s_beta 0;0 1 0 0;-s_beta 0 c_beta 0;0 0 0 1]*[c_suita -s_suita 0 0;s_suita c_suita 0 0;0 0 1 d;0 0 0 1];

Tibeta1=[1 0 0 a;0 c_alpha -s_alpha 0;0 s_alpha c_alpha 0;0 0 0 1]*[c_suita -s_suita 0 0;s_suita c_suita 0 0;0 0 1 d;0 0 0 1]*[c_beta 0 s_beta 0;0 1 0 0;-s_beta 0 c_beta 0;0 0 0 1];