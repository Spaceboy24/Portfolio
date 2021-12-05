function [t_k,t_h,t_hp,t_total] = get_init(t_k,t_h,t_hp,t_total,delta_t)

    t_h = min(t_h + delta_t,10);
    t_hp = min(t_h + delta_t,10);
    t_k = t_k + delta_t;
    t_total = t_total + delta_t

end