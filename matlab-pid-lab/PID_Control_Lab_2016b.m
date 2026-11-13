function PID_Control_Lab_2016b()
    % =========================================================================
    % PID CONTROL LAB (Compatible con MATLAB R2016b)
    % Autor: Agustín Ezequiel Feroci
    % Planta: Sistema Masa-Resorte-Amortiguador con saturación de actuador
    % =========================================================================

    % Parámetros físicos de la planta (m*x'' + b*x' + k*x = F)
    m = 1.0;        % Masa [kg]
    b = 0.8;        % Amortiguamiento [N*s/m]
    k = 2.0;        % Constante del resorte [N/m]
    u_max = 15.0;   % Saturación del actuador [N]
    dt = 0.025;     % Paso de integración [s] (40 Hz)

    % Estado inicial [posición; velocidad]
    state = [0.0; 0.0];
    integral_err = 0.0;
    t = 0.0;
    is_running = true;
    anti_windup = true;

    % Historial para gráficos (ventana de 15 segundos)
    win_len = round(15 / dt);
    t_hist  = nan(1, win_len);
    sp_hist = nan(1, win_len);
    pv_hist = nan(1, win_len);
    u_hist  = nan(1, win_len);
    p_hist  = nan(1, win_len);
    i_hist  = nan(1, win_len);
    d_hist  = nan(1, win_len);

    % --- Creación de la Ventana Clásica (Compatible R2016b) ---
    bgDark  = [0.11 0.12 0.15];
    pnlDark = [0.16 0.18 0.22];
    txtCol  = [0.92 0.94 0.96];

    fig = figure('Name', 'PID Control Lab - Interactive Simulator (R2016b)', ...
                 'NumberTitle', 'off', ...
                 'Position', [80, 80, 1180, 680], ...
                 'Color', bgDark, ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'figure', ...
                 'CloseRequestFcn', @onClose);

    % Panel Izquierdo: Controles
    pnlCtrl = uipanel('Parent', fig, 'Units', 'pixels', ...
                      'Position', [15, 15, 310, 650], ...
                      'Title', ' SINTONIZACIÓN Y PLANTA ', ...
                      'BackgroundColor', pnlDark, ...
                      'ForegroundColor', [0.4 0.8 1.0], ...
                      'FontWeight', 'bold', 'FontSize', 10);

    % Sliders compatibles con R2016b
    [sldKp, lblKp] = createSlider(pnlCtrl, 555, 'Kp (Proporcional)', 0, 25, 6.0);
    [sldKi, lblKi] = createSlider(pnlCtrl, 475, 'Ki (Integral)', 0, 15, 2.5);
    [sldKd, lblKd] = createSlider(pnlCtrl, 395, 'Kd (Derivativo)', 0, 10, 2.2);
    [sldSP, lblSP] = createSlider(pnlCtrl, 315, 'Setpoint (Objetivo [m])', -1.5, 1.5, 1.0);

    % Checkbox Anti-Windup
    chkAW = uicontrol('Parent', pnlCtrl, 'Style', 'checkbox', ...
                      'Position', [15, 265, 270, 25], ...
                      'String', ' Activar Anti-Windup (Clamping)', ...
                      'Value', 1, 'BackgroundColor', pnlDark, ...
                      'ForegroundColor', txtCol, 'FontSize', 9.5, ...
                      'FontWeight', 'bold', ...
                      'Callback', @onToggleAW);

    % Botón Perturbación
    uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
              'Position', [15, 205, 280, 42], ...
              'String', 'APLICAR PERTURBACIÓN (IMPULSO)', ...
              'BackgroundColor', [0.85 0.35 0.2], ...
              'ForegroundColor', 'white', 'FontWeight', 'bold', 'FontSize', 9.5, ...
              'Callback', @applyDisturbance);

    % Botones Pausa y Reinicio
    btnPause = uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
                         'Position', [15, 150, 135, 38], ...
                         'String', 'Pausar', ...
                         'BackgroundColor', [0.28 0.33 0.42], ...
                         'ForegroundColor', 'white', 'FontWeight', 'bold', ...
                         'Callback', @togglePause);

    uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
              'Position', [160, 150, 135, 38], ...
              'String', 'Reiniciar', ...
              'BackgroundColor', [0.28 0.33 0.42], ...
              'ForegroundColor', 'white', 'FontWeight', 'bold', ...
              'Callback', @resetSim);

    % Presets Didácticos
    uicontrol('Parent', pnlCtrl, 'Style', 'text', ...
              'Position', [15, 112, 280, 20], ...
              'String', 'Escenarios Didácticos Rápidos:', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', pnlDark, 'ForegroundColor', [0.75 0.8 0.85], ...
              'FontWeight', 'bold');

    uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
              'Position', [15, 75, 88, 32], 'String', 'Solo P', ...
              'BackgroundColor', [0.22 0.26 0.32], 'ForegroundColor', 'white', ...
              'Callback', @(~,~) setGains(10, 0, 0));

    uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
              'Position', [111, 75, 88, 32], 'String', 'Subamort.', ...
              'BackgroundColor', [0.22 0.26 0.32], 'ForegroundColor', 'white', ...
              'Callback', @(~,~) setGains(18, 8, 0.2));

    uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', ...
              'Position', [207, 75, 88, 32], 'String', 'Óptimo', ...
              'BackgroundColor', [0.22 0.26 0.32], 'ForegroundColor', 'white', ...
              'Callback', @(~,~) setGains(12, 4.5, 3.5));

    % Etiqueta de métricas en vivo
    lblMetrics = uicontrol('Parent', pnlCtrl, 'Style', 'text', ...
                           'Position', [15, 20, 280, 35], ...
                           'String', 'Error: 0.000 m | Fuerza u: 0.00 N', ...
                           'HorizontalAlignment', 'center', ...
                           'BackgroundColor', [0.12 0.14 0.17], ...
                           'ForegroundColor', [0.35 0.95 0.6], ...
                           'FontWeight', 'bold', 'FontSize', 9.5);

    % --- Panel Derecho: Ejes Gráficos Clásicos ---
    % 1. Animación Física
    axAnim = axes('Parent', fig, 'Units', 'pixels', ...
                  'Position', [380, 505, 770, 145], ...
                  'Color', pnlDark, 'XColor', txtCol, 'YColor', txtCol, ...
                  'GridColor', [0.35 0.38 0.42]);
    title(axAnim, 'Animación Física de la Planta (Masa-Resorte-Amortiguador)', 'Color', 'w', 'FontSize', 10.5);
    xlim(axAnim, [-2.2, 2.2]); ylim(axAnim, [-0.4, 1.1]);
    grid(axAnim, 'on'); set(axAnim, 'YTick', []);
    hold(axAnim, 'on');
    plot(axAnim, [-2.2 2.2], [0 0], 'Color', [0.55 0.55 0.55], 'LineWidth', 2);
    hTargetLine = plot(axAnim, [1.0 1.0], [-0.4 1.1], '--g', 'LineWidth', 1.8);
    hSpring = plot(axAnim, [-2.2, 0], [0.35, 0.35], 'Color', [0.6 0.75 0.95], 'LineWidth', 2.2);
    hCart = rectangle('Parent', axAnim, 'Position', [-0.2, 0.05, 0.4, 0.55], ...
                      'FaceColor', [0.2 0.6 0.95], 'EdgeColor', 'w', 'LineWidth', 1.5, 'Curvature', 0.15);
    hForceArrow = quiver(axAnim, 0, 0.32, 0, 0, 0, 'Color', [1 0.4 0.3], 'LineWidth', 2.5, 'MaxHeadSize', 0.5);

    % 2. Respuesta Temporal (SP vs PV)
    axResp = axes('Parent', fig, 'Units', 'pixels', ...
                  'Position', [380, 275, 770, 190], ...
                  'Color', pnlDark, 'XColor', txtCol, 'YColor', txtCol, ...
                  'GridColor', [0.35 0.38 0.42]);
    title(axResp, 'Respuesta del Sistema: Setpoint (SP) vs Variable de Proceso (PV)', 'Color', 'w', 'FontSize', 10.5);
    ylabel(axResp, 'Posición [m]', 'Color', 'w');
    ylim(axResp, [-2.0, 2.0]); grid(axResp, 'on'); hold(axResp, 'on');
    hLineSP = plot(axResp, nan, nan, '--', 'Color', [0.3 0.95 0.5], 'LineWidth', 1.8, 'DisplayName', 'Setpoint (SP)');
    hLinePV = plot(axResp, nan, nan, '-',  'Color', [0.2 0.75 1.0], 'LineWidth', 2.2, 'DisplayName', 'Posición (PV)');
    leg1 = legend(axResp, 'Location', 'northeast');
    set(leg1, 'TextColor', 'w', 'Color', [0.18 0.2 0.25]);

    % 3. Señal de Control u(t) y Desglose P, I, D
    axCtrl = axes('Parent', fig, 'Units', 'pixels', ...
                  'Position', [380, 45, 770, 190], ...
                  'Color', pnlDark, 'XColor', txtCol, 'YColor', txtCol, ...
                  'GridColor', [0.35 0.38 0.42]);
    title(axCtrl, 'Esfuerzo de Control u(t) y Desglose de Acciones P, I, D', 'Color', 'w', 'FontSize', 10.5);
    ylabel(axCtrl, 'Fuerza [N]', 'Color', 'w');
    xlabel(axCtrl, 'Tiempo [s]', 'Color', 'w');
    ylim(axCtrl, [-18, 18]); grid(axCtrl, 'on'); hold(axCtrl, 'on');
    plot(axCtrl, [0 1000], [u_max u_max], ':r', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(axCtrl, [0 1000], [-u_max -u_max], ':r', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    hLineU = plot(axCtrl, nan, nan, '-',  'Color', [1.0 0.85 0.2], 'LineWidth', 2.0, 'DisplayName', 'Control u(t)');
    hLineP = plot(axCtrl, nan, nan, '--', 'Color', [0.95 0.4 0.4], 'LineWidth', 1.1, 'DisplayName', 'P');
    hLineI = plot(axCtrl, nan, nan, '--', 'Color', [0.4 0.9 0.6],  'LineWidth', 1.1, 'DisplayName', 'I');
    hLineD = plot(axCtrl, nan, nan, '--', 'Color', [0.75 0.5 1.0], 'LineWidth', 1.1, 'DisplayName', 'D');
    leg2 = legend(axCtrl, 'Location', 'northeast', 'Orientation', 'horizontal');
    set(leg2, 'TextColor', 'w', 'Color', [0.18 0.2 0.25]);

    % --- Timer de Simulación en Tiempo Real ---
    simTimer = timer('ExecutionMode', 'fixedRate', 'Period', dt, 'TimerFcn', @stepSimulation);
    start(simTimer);

    % =========================================================================
    % FUNCIONES DE SIMULACIÓN Y CALLBACKS
    % =========================================================================
    function stepSimulation(~, ~)
        if ~is_running || ~ishandle(fig)
            return;
        end

        Kp = get(sldKp, 'Value');
        Ki = get(sldKi, 'Value');
        Kd = get(sldKd, 'Value');
        sp = get(sldSP, 'Value');
        pv = state(1);

        % Error y términos PID
        err = sp - pv;
        P_term = Kp * err;
        D_term = -Kd * state(2); % Derivada sobre la medición (sin derivative kick)

        % Integración con Anti-Windup (Clamping)
        candidate_integral = integral_err + err * dt;
        I_term = Ki * candidate_integral;
        u_unsat = P_term + I_term + D_term;

        if anti_windup
            if abs(u_unsat) > u_max && sign(err) == sign(u_unsat)
                I_term = Ki * integral_err;
            else
                integral_err = candidate_integral;
            end
        else
            integral_err = candidate_integral;
        end

        % Saturación del actuador
        u = max(min(P_term + I_term + D_term, u_max), -u_max);

        % Integración numérica RK4
        f = @(x) [x(2); (u - b*x(2) - k*x(1)) / m];
        k1 = f(state);
        k2 = f(state + 0.5*dt*k1);
        k3 = f(state + 0.5*dt*k2);
        k4 = f(state + dt*k3);
        state = state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        t = t + dt;

        % Actualizar buffers
        t_hist  = [t_hist(2:end), t];
        sp_hist = [sp_hist(2:end), sp];
        pv_hist = [pv_hist(2:end), state(1)];
        u_hist  = [u_hist(2:end), u];
        p_hist  = [p_hist(2:end), P_term];
        i_hist  = [i_hist(2:end), I_term];
        d_hist  = [d_hist(2:end), D_term];

        % Actualizar animación y gráficos
        pos = state(1);
        set(hTargetLine, 'XData', [sp sp]);
        set(hCart, 'Position', [pos - 0.2, 0.05, 0.4, 0.55]);

        x_spring = linspace(-2.2, pos - 0.2, 16);
        y_spring = 0.32 + 0.11 * sin(linspace(0, 7*pi, 16));
        y_spring(1) = 0.32; y_spring(end) = 0.32;
        set(hSpring, 'XData', x_spring, 'YData', y_spring);

        set(hForceArrow, 'XData', pos, 'YData', 0.32, 'UData', u * 0.06, 'VData', 0);

        set(hLineSP, 'XData', t_hist, 'YData', sp_hist);
        set(hLinePV, 'XData', t_hist, 'YData', pv_hist);
        set(hLineU,  'XData', t_hist, 'YData', u_hist);
        set(hLineP,  'XData', t_hist, 'YData', p_hist);
        set(hLineI,  'XData', t_hist, 'YData', i_hist);
        set(hLineD,  'XData', t_hist, 'YData', d_hist);

        if t > 15
            xlim(axResp, [t - 15, t]);
            xlim(axCtrl, [t - 15, t]);
        else
            xlim(axResp, [0, 15]);
            xlim(axCtrl, [0, 15]);
        end

        set(lblMetrics, 'String', sprintf('Error: %+.3f m  |  Fuerza u: %+.2f N', err, u));
        drawnow limitrate;
    end

    function [sld, lbl] = createSlider(parent, yPos, name, minV, maxV, initV)
        lbl = uicontrol('Parent', parent, 'Style', 'text', ...
                        'Position', [15, yPos + 24, 280, 20], ...
                        'String', sprintf('%s: %.2f', name, initV), ...
                        'HorizontalAlignment', 'left', ...
                        'BackgroundColor', pnlDark, 'ForegroundColor', 'white', ...
                        'FontWeight', 'bold', 'FontSize', 9.5);
        sld = uicontrol('Parent', parent, 'Style', 'slider', ...
                        'Position', [15, yPos, 280, 22], ...
                        'Min', minV, 'Max', maxV, 'Value', initV, ...
                        'Callback', @(src,~) set(lbl, 'String', sprintf('%s: %.2f', name, get(src, 'Value'))));
    end

    function setGains(kp, ki, kd)
        set(sldKp, 'Value', kp); set(lblKp, 'String', sprintf('Kp (Proporcional): %.2f', kp));
        set(sldKi, 'Value', ki); set(lblKi, 'String', sprintf('Ki (Integral): %.2f', ki));
        set(sldKd, 'Value', kd); set(lblKd, 'String', sprintf('Kd (Derivativo): %.2f', kd));
    end

    function applyDisturbance(~, ~)
        state(2) = state(2) + 3.5;
    end

    function onToggleAW(src, ~)
        anti_windup = logical(get(src, 'Value'));
    end

    function togglePause(~, ~)
        is_running = ~is_running;
        if is_running
            set(btnPause, 'String', 'Pausar');
        else
            set(btnPause, 'String', 'Reanudar');
        end
    end

    function resetSim(~, ~)
        state = [0.0; 0.0];
        integral_err = 0.0;
        t = 0.0;
        t_hist(:) = nan; sp_hist(:) = nan; pv_hist(:) = nan;
        u_hist(:) = nan; p_hist(:) = nan; i_hist(:) = nan; d_hist(:) = nan;
    end

    function onClose(~, ~)
        try
            stop(simTimer);
            delete(simTimer);
        catch
        end
        delete(fig);
    end
end