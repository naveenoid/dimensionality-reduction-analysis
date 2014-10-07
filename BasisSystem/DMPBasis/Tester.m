function Tester( ResponseProfiles, U , tmax)
%TESTER Function to perform the learning of the DMP bases extracted by IBE.
% Performs least square fit of weights using a computed pseudo-inverse and
% then plots a comparison of the learned and computed basis.
response = ResponseProfiles;

t = linspace(0, tmax, size(response, 2));
u = U(t);

resp = response(1 : end - 2, :);
w = u * resp' * inv(resp * resp');
w = [w, 0, 0];

figure;
subplot(311), hold on;
plot(t, u, 'k');
plot(t, w * response, 'r');
legend({'given u(t)', 'fit based on responses'});
subplot(312), hold on;
for i = 1 : size(response, 1) - 2
    plot(t, response(i, :), 'LineWidth', abs(w(i)));
end
subplot(313), hold on;
    bar(w);


end

