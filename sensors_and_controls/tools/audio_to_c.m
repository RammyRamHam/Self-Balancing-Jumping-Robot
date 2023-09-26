format long g
audioOut = "";

[audio, audio_fs] = audioread("./Resources/test_audio2.wav");


numSamples = length(audio);
maxOutVal = 2^8-1;

audio = audio/max(audio);

audiotest = zeros(numSamples, 1);
for i=1:numSamples
    audiotest(i) = floor(maxOutVal*(audio(i, 1) + 1)/2);
    audioOut = audioOut + sprintf("%d, ", floor(maxOutVal*(audio(i, 1) + 1)/2)) + sprintf("%d, ", floor(maxOutVal*(audio(i, 1) + 1)/2));
    if mod(i, 10) == 0
        audioOut = audioOut + "\n";
    end
        
end

sprintf(audioOut)