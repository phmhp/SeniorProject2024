package com.example.roverosong

import OpenAIService
import android.app.Activity
import android.app.Dialog
import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.speech.RecognizerIntent
import android.speech.tts.TextToSpeech
import android.view.Gravity
import android.view.LayoutInflater
import android.widget.*
import retrofit2.*
import retrofit2.converter.gson.GsonConverterFactory
import java.util.*

class ChatActivity : BaseActivity(), TextToSpeech.OnInitListener {

    private lateinit var micButton: ImageButton
    private lateinit var responseText: TextView
    private lateinit var userSpeechText: TextView
    private lateinit var tts: TextToSpeech
    private lateinit var helpButton: Button
    private lateinit var backButton: ImageButton

    private val apiKey = BuildConfig.OPENAI_API_KEY  // OpenAI API 키 가져오기
    private val retrofit = Retrofit.Builder()
        .baseUrl("https://api.openai.com/")
        .addConverterFactory(GsonConverterFactory.create())
        .build()
    private val openAIService = retrofit.create(OpenAIService::class.java)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_chat)

        micButton = findViewById(R.id.micButton)
        responseText = findViewById(R.id.responseText)
        userSpeechText = findViewById(R.id.userSpeechText)
        helpButton = findViewById(R.id.helpButton)
        backButton = findViewById(R.id.backButton)  // 버튼 연결

        // 뒤로 가기 버튼 클릭 시 MainActivity로 이동
        backButton.setOnClickListener {
            val intent = Intent(this, MainActivity::class.java)
            startActivity(intent)
            finish()
        }

        // TTS 엔진 초기화
        tts = TextToSpeech(this, this)

        // ? 버튼 클릭 시 팝업 열기
        helpButton.setOnClickListener {
            showQuestionsPopup()
        }

        // 음성 인식 버튼 클릭 시 실행
        micButton.setOnClickListener {
            startVoiceRecognition()
        }
    }

    // '이렇게 질문해보세요!' 팝업 띄우기
    private fun showQuestionsPopup() {
        val dialog = Dialog(this)
        val view = LayoutInflater.from(this).inflate(R.layout.questions_popup, null)
        dialog.setContentView(view)

        val closeButton = view.findViewById<Button>(R.id.closePopupButton)
        closeButton.setOnClickListener {
            dialog.dismiss()
        }

        dialog.window?.setLayout(
            (resources.displayMetrics.widthPixels * 0.85).toInt(),
            (resources.displayMetrics.heightPixels * 0.4).toInt()
        )
        dialog.window?.setGravity(Gravity.CENTER)
        dialog.window?.setBackgroundDrawableResource(android.R.color.transparent)
        dialog.show()
    }

    // 음성 인식 실행
    private fun startVoiceRecognition() {
        val intent = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH)
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "ko-KR")
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "말씀해주세요...")

        try {
            startActivityForResult(intent, 100)
        } catch (e: Exception) {
            Toast.makeText(this, "음성 인식을 사용할 수 없습니다.", Toast.LENGTH_SHORT).show()
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == 100 && resultCode == Activity.RESULT_OK) {
            val result = data?.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS)
            val recognizedText = result?.get(0) ?: ""

            userSpeechText.text = "🙋‍♂️ 사용자: $recognizedText"

            // OpenAI API 호출
            sendQuestionToOpenAI(recognizedText)
        }
    }

    private fun sendQuestionToOpenAI(question: String) {
            responseText.text = "로봇이 생각 중..."

            val request = OpenAIRequest(
                messages = listOf(
                    Message("system", "당신은 roverosong(로버로송)이라는 실내 자율주행 안내로봇의 AI입니다. " +
                            "간단한 인사말과 자기소개를 할 수 있습니다." + "매번 인사말과 자기소개를 할 필요는 없습니다." +
                            "roverosong은 사용자를 목적지로 안내하는 역할을 수행합니다. " + "센서들로 주변을 보고, 자기 위치를 파악하고, 정해진 길을 따라 가다가 장애물을 만나면 피해서 목적지까지 갑니다." +
                            "각 목적지까지 이동하는 데 약 3~5분 정도 소요됩니다. " +
                            "길 안내 요청을 받으면, '메인 화면에서 원하는 목적지를 선택해주세요.'라고 안내해야 합니다. " +
                            "안내 도중 사용자가 안내를 중지하고 싶다면, 앱에서 '안내중지' 버튼을 누르면 됩니다. " +
                            "사용자의 질문이 이해되지 않는다면 '저는 roverosong 로봇과 관련된 정보만 제공할 수 있습니다.'라고 답변하세요."+
                            "한 문장을 말할 때 마다 다음 문장으로 넘겨서 출력된 문장들이 보기 좋게 해주세요." ),
                    Message("user", question)
                )
            )

        val apiKey = "Bearer ${BuildConfig.OPENAI_API_KEY}"  // 🔹 Bearer 토큰 형식으로 API 키 설정

        openAIService.getChatCompletion(apiKey, request).enqueue(object : Callback<OpenAIResponse> {
            override fun onResponse(call: Call<OpenAIResponse>, response: Response<OpenAIResponse>) {
                if (response.isSuccessful) {
                    val chatResponse = response.body()
                    val answer = chatResponse?.choices?.firstOrNull()?.message?.content ?: "응답을 받을 수 없습니다."

                    responseText.text = answer
                    speakAnswer(answer) // TTS로 답변 읽어주기
                } else {
                    responseText.text = "오류 발생: API 응답 실패"

                    Log.e("OpenAI", "API 응답 실패: ${response.code()} - ${response.message()}")
                    Log.e("OpenAI", "응답 바디: ${response.errorBody()?.string()}")
                }
            }

            override fun onFailure(call: Call<OpenAIResponse>, t: Throwable) {
                responseText.text = "네트워크 오류 발생: ${t.message}"

                Log.e("OpenAI", "네트워크 오류: ${t.message}")
            }
        })
    }


    override fun onInit(status: Int) {
        if (status == TextToSpeech.SUCCESS) {
            tts.language = Locale.KOREAN
        }
    }

    private fun speakAnswer(answer: String) {
        val params = Bundle()
        tts.speak(answer, TextToSpeech.QUEUE_FLUSH, params, null)
    }
    override fun onDestroy() {
        if (::tts.isInitialized) {
            tts.stop()
            tts.shutdown()
        }
        super.onDestroy()
    }
}
