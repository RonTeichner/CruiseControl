function [x,sin_theta,roadZ] = roadAngleGen(distance)
% Inputs:

% I want a hill up and down to be 2Km. so a complete cycle of 4Km
% so max freq is 1/4000 [cycles@meter]
maxFreq = 1/2000; % [cycles@meter]
fs = 8*maxFreq; % [hz]
ts = 1/fs; % sample every 'ts' meters.
peakHeight = 400; % [m]


Wn = maxFreq / (fs/2);

N = 20;
b = fir1(N,Wn);
%  B = fir1(N,Wn);  Wn must be between 0 < Wn < 1.0, with 1.0 corresponding to half the sample rate.
[H,W] = freqz(b,1,[],fs);

figure; 
%subplot(2,1,1); 
plot(W*100,20*log10(abs(H)));
hold all; stem(maxFreq*100 , min(20*log10(abs(H)))-10);
xlabel('cycles @ 100 meter'); ylabel('gain [db]'); title('filter response'); grid on;
%ylim([-100,0]);

nSamples = ceil(distance*fs);
x = transpose([0:(nSamples - 1)]./fs); % [m]

whiteNoise = peakHeight*randn(max(10*numel(x),10*N),1);
roadZ = filter(b,1,whiteNoise); % [m]
roadZ = roadZ(end-nSamples+1:end);
theta_rad = atan([0 ; diff(roadZ)]/ts);
sin_theta = sin(theta_rad);

nfft = numel(roadZ);
fVec = [-fs/2:fs/nfft:(fs/2 - fs/nfft)]; % [cycles@meter]
roadZFft = 20*log10(abs(fftshift(fft(roadZ))));
roadZFft = roadZFft - max(roadZFft);

[~,fVecZeroIdx] = min(abs(fVec));
%subplot(2,1,2); 
hold all; plot(fVec(fVecZeroIdx:end)*100,roadZFft(fVecZeroIdx:end)); %xlabel('cycles @ 100 meter'); ylabel('[db]'); grid on; title('roadZ fft');
%hold all; stem(maxFreq*100 , min(roadZFft)-10); %legend('roadZ fft','cutoff freq'); % ylim([-100,0]);
legend('filter response','cutoff freq','road fft');
end

